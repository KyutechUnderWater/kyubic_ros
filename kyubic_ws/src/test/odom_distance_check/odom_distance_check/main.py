"""Read-only tool to check odom integration accuracy against a known moved distance.

The operator marks points in time (Enter key) while physically moving the vehicle or
sensor a known distance between marks. Each interval between two marks is recorded to a
CSV file as a position delta read from /localization/odom, to be compared by hand against
the distance the operator actually measured (e.g. with a tape measure).

This node never creates a publisher or service client. It cannot send any command to the
vehicle. Do not add any of those here without an explicit safety review (see
sample/blue_control for the same design principle applied to a different diagnostic node).
"""

import csv
import datetime
import math
import threading
from pathlib import Path

from common_msgs.msg import Status
from localization_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

CSV_HEADER = [
    "segment_index",
    "start_stamp_sec",
    "start_stamp_nanosec",
    "end_stamp_sec",
    "end_stamp_nanosec",
    "start_x",
    "start_y",
    "start_z_depth",
    "end_x",
    "end_y",
    "end_z_depth",
    "dx",
    "dy",
    "distance_m",
    "dz_depth",
    "elapsed_time_s",
    "worst_depth_status",
    "worst_imu_status",
    "worst_dvl_status",
    "cumulative_distance_from_origin_m",
    "note",
]

# common_msgs/msg/Status: NORMAL=0, WARNING=1, ERROR=2. Larger value = worse.
_STATUS_NAME = {
    Status.NORMAL: "NORMAL",
    Status.WARNING: "WARNING",
    Status.ERROR: "ERROR",
}

_END_COMMANDS = {"done", "q", "quit", "exit"}


class Mark:
    """A single operator-marked point: odom snapshot plus status observed since previous mark."""

    def __init__(self, odom: Odometry, worst_depth: int, worst_imu: int, worst_dvl: int) -> None:
        """Store the odom snapshot and the worst per-sensor status seen since the previous mark."""
        self.odom = odom
        self.worst_depth = worst_depth
        self.worst_imu = worst_imu
        self.worst_dvl = worst_dvl


class OdomDistanceCheckNode(Node):
    """Subscribes to odom and tracks the latest message plus worst status since the last mark.

    Holds no publishers or service clients: it cannot actuate the vehicle.
    """

    def __init__(self) -> None:
        """Declare parameters, subscribe to odom, and initialize status tracking state."""
        super().__init__("odom_distance_check")

        self.declare_parameter("odom_topic", "/localization/odom")
        self.declare_parameter("output_dir", "~/odom_distance_check_logs")

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()

        self._lock = threading.Lock()
        self._latest_odom: Odometry | None = None
        self._worst_depth = Status.NORMAL
        self._worst_imu = Status.NORMAL
        self._worst_dvl = Status.NORMAL

        self._odom_subscription = self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, 10
        )

        self.get_logger().info(
            f"odom_distance_check started (subscribe-only): odom_topic={self._odom_topic}"
        )

    @property
    def output_dir(self) -> Path:
        """Directory (already expanded, e.g. ~) that log files should be written under."""
        return self._output_dir

    def _odom_callback(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg
            self._worst_depth = max(self._worst_depth, msg.status.depth.id)
            self._worst_imu = max(self._worst_imu, msg.status.imu.id)
            self._worst_dvl = max(self._worst_dvl, msg.status.dvl.id)

    def take_mark(self) -> Mark | None:
        """Snapshot the latest odom and the worst status since the previous mark, then reset it.

        Returns None if no odom message has been received yet.
        """
        with self._lock:
            if self._latest_odom is None:
                return None
            mark = Mark(self._latest_odom, self._worst_depth, self._worst_imu, self._worst_dvl)
            # 次の区間はこのマークの直後から観測を再開する(このマーク自身のstatusを初期値にする)。
            self._worst_depth = self._latest_odom.status.depth.id
            self._worst_imu = self._latest_odom.status.imu.id
            self._worst_dvl = self._latest_odom.status.dvl.id
            return mark


def _status_name(status_id: int) -> str:
    return _STATUS_NAME.get(status_id, f"UNKNOWN({status_id})")


def _stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _make_csv_path(output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return output_dir / f"odom_distance_check_{timestamp}.csv"


def _init_csv(csv_path: Path) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        csv.writer(file).writerow(CSV_HEADER)


def _build_segment_row(
    segment_index: int, origin: Mark, start: Mark, end: Mark, note: str
) -> list:
    start_pos = start.odom.pose.position
    end_pos = end.odom.pose.position
    origin_pos = origin.odom.pose.position

    dx = end_pos.x - start_pos.x
    dy = end_pos.y - start_pos.y
    dz_depth = end_pos.z_depth - start_pos.z_depth
    distance_m = math.hypot(dx, dy)
    elapsed_time_s = _stamp_to_seconds(end.odom.header.stamp) - _stamp_to_seconds(
        start.odom.header.stamp
    )
    cumulative_distance_from_origin_m = math.hypot(
        end_pos.x - origin_pos.x, end_pos.y - origin_pos.y
    )

    return [
        segment_index,
        start.odom.header.stamp.sec,
        start.odom.header.stamp.nanosec,
        end.odom.header.stamp.sec,
        end.odom.header.stamp.nanosec,
        start_pos.x,
        start_pos.y,
        start_pos.z_depth,
        end_pos.x,
        end_pos.y,
        end_pos.z_depth,
        dx,
        dy,
        distance_m,
        dz_depth,
        elapsed_time_s,
        _status_name(end.worst_depth),
        _status_name(end.worst_imu),
        _status_name(end.worst_dvl),
        cumulative_distance_from_origin_m,
        note,
    ]


def _print_segment_summary(segment_index: int, row: list) -> None:
    dx, dy, distance_m, dz_depth, elapsed_time_s = row[11], row[12], row[13], row[14], row[15]
    worst_depth, worst_imu, worst_dvl = row[16], row[17], row[18]
    cumulative = row[19]
    print(
        f"[segment {segment_index}] dx={dx:+.3f} dy={dy:+.3f} distance={distance_m:.3f}m "
        f"dz_depth={dz_depth:+.3f} elapsed={elapsed_time_s:.1f}s "
        f"worst(depth/imu/dvl)={worst_depth}/{worst_imu}/{worst_dvl} "
        f"cumulative_from_origin={cumulative:.3f}m"
    )
    if worst_depth != "NORMAL" or worst_imu != "NORMAL" or worst_dvl != "NORMAL":
        print(
            "  ⚠ この区間はセンサーstatus異常を含みます。結果は参考値として扱ってください"
            " (README参照)。"
        )


def _print_final_summary(rows: list) -> None:
    if not rows:
        print("記録された区間はありません。")
        return
    print("\n=== 記録サマリー ===")
    total_distance = 0.0
    for row in rows:
        segment_index, distance_m = row[0], row[13]
        total_distance += distance_m
        print(f"  segment {segment_index}: distance={distance_m:.3f}m")
    print(f"  区間の合計移動距離(直線距離の総和): {total_distance:.3f}m")
    print(f"  最終区間の原点からの累積直線距離: {rows[-1][19]:.3f}m")


def _run_interactive_loop(node: OdomDistanceCheckNode, csv_path: Path) -> None:
    print("odom_distance_check: 記録を開始します。")
    print("  空行(Enter)またはメモ文字列の入力 -> マーク")
    print("  'done'/'q'/'quit'/'exit' (またはCtrl+C/Ctrl+D) -> 終了")
    print("最初のマークは原点として記録されます(まだodomを受信していない場合は待機してください)。")

    origin: Mark | None = None
    previous: Mark | None = None
    segment_index = 0
    rows: list = []

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if line.lower() in _END_COMMANDS:
            break

        mark = node.take_mark()
        if mark is None:
            print("  まだodomを受信していません。記録をスキップしました。")
            continue

        if previous is None:
            origin = mark
            previous = mark
            print("  [mark 0] 原点として記録しました。")
            continue

        segment_index += 1
        row = _build_segment_row(segment_index, origin, previous, mark, note=line)
        with csv_path.open("a", newline="", encoding="utf-8") as file:
            csv.writer(file).writerow(row)
        rows.append(row)
        _print_segment_summary(segment_index, row)
        previous = mark

    _print_final_summary(rows)
    print(f"\nCSVファイル: {csv_path}")


def main(args: list[str] | None = None) -> None:
    """Run the odom_distance_check node until the operator ends the session."""
    rclpy.init(args=args)
    node = OdomDistanceCheckNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    csv_path = _make_csv_path(node.output_dir)
    _init_csv(csv_path)

    try:
        _run_interactive_loop(node, csv_path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
