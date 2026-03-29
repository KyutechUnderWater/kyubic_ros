import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from localization_msgs.msg import Odometry
from sensor_msgs.msg import Image

import depthai as dai
import time
from pathlib import Path
import threading
import subprocess
import os
import csv
import cv2
import numpy as np
import queue  # 追加: 非同期処理用


class HeadlessOakCameraNode(Node):
    """
    ROS 2トピックをトリガーとして動作する、ヘッドレス（GUIなし）のOAKカメラノード。
    - 常時映像をROSトピックに配信 (Live View)
    - トリガー時のみ画像をディスクに保存 (Mapping)
    - 保存処理は別スレッドで非同期実行 (Non-blocking)
    """

    def __init__(self):
        # 1. ROS 2 ノードの初期化
        super().__init__("headless_oak_camera_node")

        # 2. 保存設定と状態管理
        run_timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.save_dir = Path(f"DATA_{run_timestamp}")
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.image_count = 1
        self.is_recording = False
        self.h265_file_handle = None
        self.h265_filepath = None

        # ★重要: 画像保存用のキュー (サイズ50でバッファリング)
        self.save_queue = queue.Queue(maxsize=50)

        self.get_logger().info(f"データ保存先フォルダ: {self.save_dir.resolve()}")

        # 3. DepthAI パイプラインの構築
        self.pipeline = self.create_pipeline()

        # 4. デバイスの初期化とキューの取得
        try:
            self.device = dai.Device(self.pipeline)
            self.setup_queues()
            self.get_logger().info("OAK-1カメラの準備ができました。")
        except Exception as e:
            self.get_logger().error(f"OAK-1カメラの初期化に失敗: {e}")
            raise

        # 5. 起動時に固定カメラ設定を適用
        self.apply_initial_camera_settings()

        # データ保持用の変数を Odometry 型で初期化
        self.current_odom: Odometry = Odometry()

        # 6. ROS 2 サブスクライバ/パブリッシャの作成
        # GNSS情報の購読
        self.photo_sub = self.create_subscription(
            Odometry, "/localization/odom", self.photo_trigger_callback, 10
        )
        # 動画制御
        self.video_start_sub = self.create_subscription(
            String, "trigger_video_start", self.video_start_callback, 10
        )
        self.video_stop_sub = self.create_subscription(
            String, "trigger_video_stop", self.video_stop_callback, 10
        )

        # Image型のパブリッシャー
        self.image_pub = self.create_publisher(Image, "/camera/bottom", 10)

        # 7. スレッド管理
        self.stop_threads_event = threading.Event()
        self.capture_still_event = threading.Event()

        # (A) 画像取得＆パブリッシュ用スレッド (高速ループ)
        self.still_thread = threading.Thread(target=self.still_capture_loop)
        # (B) 画像保存用スレッド (低速ループ・I/O担当)
        self.save_thread = threading.Thread(target=self.image_save_loop)
        # (C) 動画保存用スレッド
        self.video_thread = threading.Thread(target=self.video_loop)

        self.still_thread.start()
        self.save_thread.start()
        self.video_thread.start()

        # 8. CSVファイルの準備
        self.setup_csv_logger()

        self.get_logger().info("ノード準備完了。常時映像配信中。トリガー待機中...")
        self.get_logger().info("終了するには Ctrl+C を押してください。")

    def create_pipeline(self):
        """DepthAIパイプラインを構築する"""
        pipeline = dai.Pipeline()

        # --- ノードの作成 ---
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        video_enc = pipeline.create(dai.node.VideoEncoder)

        # カメラ制御用の入力ノード
        control_in = pipeline.create(dai.node.XLinkIn)
        control_in.setStreamName("control")

        # --- 出力ノード ---
        xout_video_encoded = pipeline.create(dai.node.XLinkOut)
        xout_video_encoded.setStreamName("h265")

        xout_still_uncompressed = pipeline.create(dai.node.XLinkOut)
        xout_still_uncompressed.setStreamName("still")

        # --- カメラ設定 ---
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        # 12MPの高解像度設定
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        cam_rgb.setFps(30)
        cam_rgb.setInterleaved(False)

        # --- エンコーダ設定 ---
        video_enc.setDefaultProfilePreset(
            cam_rgb.getFps(), dai.VideoEncoderProperties.Profile.H265_MAIN
        )

        # --- パイプラインの接続 ---
        # データの詰まりを防ぐためのノンブロッキング設定
        xout_still_uncompressed.input.setBlocking(False)
        xout_still_uncompressed.input.setQueueSize(10)  # バッファを少し増やす

        xout_video_encoded.input.setBlocking(False)
        xout_video_encoded.input.setQueueSize(30)

        cam_rgb.video.link(video_enc.input)
        cam_rgb.video.link(xout_still_uncompressed.input)
        video_enc.bitstream.link(xout_video_encoded.input)
        control_in.out.link(cam_rgb.inputControl)

        return pipeline

    def setup_queues(self):
        """デバイスから入出力キューを取得する"""
        self.h265_queue = self.device.getOutputQueue(name="h265", maxSize=30, blocking=False)
        self.still_queue = self.device.getOutputQueue(name="still", maxSize=10, blocking=False)
        self.control_queue = self.device.getInputQueue("control")

    def apply_initial_camera_settings(self):
        """起動時にカメラの固定設定を適用する"""
        self.get_logger().info("ハードコードされたカメラ設定を適用しています...")

        # --- ここでパラメータを編集 ---
        exp_time = 10000  # 露光時間 (us), 1-33000
        iso = 700  # ISO感度, 100-1600
        focus = 135  # フォーカス値 (0-255, 遠いほど小さい)
        wb_temp = 5000  # ホワイトバランス (K), 1000-12000
        # -------------------------

        ctrl = dai.CameraControl()
        ctrl.setManualExposure(exp_time, iso)
        ctrl.setManualFocus(focus)
        ctrl.setManualWhiteBalance(wb_temp)

        self.control_queue.send(ctrl)
        self.get_logger().info(
            f"設定完了 - 露光: {exp_time}us, ISO: {iso}, フォーカス: {focus}, WB: {wb_temp}K"
        )

    def setup_csv_logger(self):
        """位置情報ロギング用のCSVファイルを準備する"""
        self.filename = "global_pose.csv"
        self.csv_file = open(self.save_dir / self.filename, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "image_path",
                "header_timestamp_sec",
                "header_timestamp_nanosec",
                "header_frame_id",
                "status_depth",
                "status_imu",
                "status_dvl",
                "coordinate_system_id",
                "ref_pose_latitude",
                "ref_pose_longitude",
                "ref_pose_plane_x",
                "ref_pose_plane_y",
                "ref_pose_meridian_convergence",
                "current_pose_latitude",
                "current_pose_longitude",
                "current_pose_plane_x",
                "current_pose_plane_y",
                "current_pose_meridian_convergence",
                "azimuth",
                "depth",
                "altitude",
            ]
        )
        self.get_logger().info(f"位置情報は {self.filename} に記録されます。")

    # --- ROSコールバック関数群 ---
    def photo_trigger_callback(self, msg: Odometry):
        # ここではフラグを立てるだけ（最速で抜ける）
        self.current_odom = msg
        self.capture_still_event.set()
        # ログ過多を防ぐため、ここではログを出さないかdebugにする
        # self.get_logger().debug("トリガー受信")

    def video_start_callback(self, msg):
        if self.is_recording:
            self.get_logger().warn("すでに録画中です。")
            return
        self.is_recording = True
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.h265_filepath = self.save_dir / f"video_{timestamp}.h265"
        self.h265_file_handle = open(self.h265_filepath, "wb")
        self.get_logger().info(f"動画撮影を開始しました: {self.h265_filepath}")

    def video_stop_callback(self, msg):
        if not self.is_recording:
            self.get_logger().warn("録画中ではありません。")
            return
        self.is_recording = False
        if self.h265_file_handle:
            self.h265_file_handle.close()
            self.h265_file_handle = None
        self.get_logger().info("録画を停止しました。MP4への変換を開始します...")
        threading.Thread(target=self.convert_to_mp4).start()

    def convert_to_mp4(self):
        if not self.h265_filepath or not self.h265_filepath.exists():
            return
        mp4_filepath = self.h265_filepath.with_suffix(".mp4")
        command = [
            "ffmpeg",
            "-framerate",
            "30",
            "-i",
            str(self.h265_filepath),
            "-c",
            "copy",
            "-y",
            str(mp4_filepath),
        ]
        try:
            subprocess.run(
                command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            self.get_logger().info(f"MP4変換成功: {mp4_filepath}")
            os.remove(self.h265_filepath)
        except Exception as e:
            self.get_logger().error(f"動画変換失敗: {e}")

    # --- ★ スレッド処理 (Producer: 取得＆配信) ---
    def still_capture_loop(self):
        """
        カメラから画像を取得し、ROSトピックへ配信（間引きあり）。
        トリガーがある場合のみ、保存キューへデータを送る。
        """
        loop_counter = 0  # カウンタを追加

        while not self.stop_threads_event.is_set():
            in_still = self.still_queue.tryGet()

            if in_still is not None:
                now = self.get_clock().now()
                frame = in_still.getCvFrame()  # OpenCV形式

                # 1. 常時パブリッシュ (Live View) - ★ここを修正★
                # 30fpsのカメラなら、6回に1回送れば 5fps になる
                if loop_counter % 30 == 0:
                    self.publish_image(frame, now)

                loop_counter += 1

                self.get_logger().info(f"ループカウンタ: {loop_counter}K")

                # 2. トリガー確認 -> 保存キューへ (Logging)
                if self.capture_still_event.is_set():
                    self.capture_still_event.clear()

                    save_data = {
                        "frame": frame.copy(),
                        "odom": self.current_odom,
                        "ros_time": now,
                        "count": self.image_count,
                    }
                    self.image_count += 1

                    try:
                        self.save_queue.put(save_data, block=False)
                        self.get_logger().info(
                            f"トリガー検知 -> 保存キューへ登録 (No.{save_data['count']})"
                        )
                    except queue.Full:
                        self.get_logger().warn(
                            "【警告】保存キューが満杯です！画像をドロップしました。"
                        )
            else:
                time.sleep(0.001)

    # --- ★ スレッド処理 (Consumer: ディスク保存) ---
    def image_save_loop(self):
        """保存キューからデータを取り出し、ファイル書き込みを行う"""
        while not self.stop_threads_event.is_set():
            try:
                # データが来るまで待機 (timeout付きで無限ブロック回避)
                data = self.save_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # データ取り出し
            frame = data["frame"]
            odom = data["odom"]
            ros_time = data["ros_time"]
            count = data["count"]

            # ファイル名生成
            ts_str = (
                f"{ros_time.nanoseconds // 1_000_000_000}.{ros_time.nanoseconds % 1_000_000_000}"
            )
            filename = f"image{count}_{ts_str}.jpg"
            save_filepath = self.save_dir / filename

            # 1. ディスクへの書き込み (これが重い処理)
            try:
                cv2.imwrite(str(save_filepath), frame)
            except Exception as e:
                self.get_logger().error(f"画像保存失敗: {e}")
                self.save_queue.task_done()
                continue

            # 2. CSVへの書き込み
            try:
                header = odom.header
                status = odom.status
                odom_pose = odom.pose
                g_pos = odom_pose.global_pos
                depth_val = 0.0
                altitude_val = 0.0

                self.csv_writer.writerow(
                    [
                        filename,
                        header.stamp.sec,
                        header.stamp.nanosec,
                        header.frame_id,
                        status.depth.id,
                        status.imu.id,
                        status.dvl.id,
                        g_pos.coordinate_system_id,
                        g_pos.ref_pose.latitude,
                        g_pos.ref_pose.longitude,
                        g_pos.ref_pose.plane_x,
                        g_pos.ref_pose.plane_y,
                        g_pos.ref_pose.meridian_convergence,
                        g_pos.current_pose.latitude,
                        g_pos.current_pose.longitude,
                        g_pos.current_pose.plane_x,
                        g_pos.current_pose.plane_y,
                        g_pos.current_pose.meridian_convergence,
                        g_pos.azimuth,
                        depth_val,
                        altitude_val,
                    ]
                )
                self.csv_file.flush()
            except Exception as e:
                self.get_logger().error(f"CSV書き込み失敗: {e}")

            # 完了ログ
            # self.get_logger().info(f"保存完了: {filename} (Queue残: {self.save_queue.qsize()})")

            # キューのタスク完了通知
            self.save_queue.task_done()

    def publish_image(self, frame, ros_time):
        """画像をリサイズし、sensor_msgs/Image としてパブリッシュする"""
        # エラーハンドリングは最小限にして、バグがあれば落とす
        h, w = frame.shape[:2]
        target_w = 1280  # 配信用の幅

        if w > target_w:
            scale = target_w / w
            target_h = int(h * scale)
            frame_for_topic = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)
        else:
            frame_for_topic = frame
            target_h = h

        msg = Image()
        msg.header.stamp = ros_time.to_msg()
        msg.header.frame_id = "oak_rgb_camera_link"
        msg.height = target_h
        msg.width = target_w
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = target_w * 3
        msg.data = frame_for_topic.tobytes()

        self.image_pub.publish(msg)

    def video_loop(self):
        """動画データを保存するスレッド"""
        while not self.stop_threads_event.is_set():
            if self.h265_queue is None:
                time.sleep(0.1)
                continue

            h265_packet = self.h265_queue.tryGet()
            if h265_packet is not None and self.is_recording and self.h265_file_handle:
                h265_packet.getData().tofile(self.h265_file_handle)
            else:
                time.sleep(0.001)

    def destroy_node(self):
        """終了処理"""
        self.get_logger().info("ノードをシャットダウンします...")
        self.stop_threads_event.set()

        if self.is_recording:
            self.video_stop_callback(String())

        self.get_logger().info("Waiting for threads to finish...")
        self.still_thread.join()
        self.save_thread.join()
        self.video_thread.join()

        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()

        if hasattr(self, "device"):
            self.device.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeadlessOakCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"予期せぬエラー: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
