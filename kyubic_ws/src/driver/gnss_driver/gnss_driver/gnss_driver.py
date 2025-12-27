import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from driver_msgs.msg import Gnss
import sys
import socket
import pynmea2


class GnssPublisher(Node):
    def __init__(self):
        super().__init__("gnss_publisher_node")

        # パラメータを宣言
        self.declare_parameter("tcp_ip", "192.168.9.20")
        self.declare_parameter("tcp_port", 5050)
        self.declare_parameter("hdop_error_factor", 2.0)  # HDOPを分散に変換する係数

        # パラメータを取得
        self.tcp_ip = self.get_parameter("tcp_ip").get_parameter_value().string_value
        self.tcp_port = self.get_parameter("tcp_port").get_parameter_value().integer_value
        self.hdop_error_factor = (
            self.get_parameter("hdop_error_factor").get_parameter_value().double_value
        )

        # Publisherを作成
        sensor_qos = rclpy.qos.qos_profile_sensor_data
        # self.fix_publisher_ = self.create_publisher(NavSatFix, "/gps/fix", 10)
        # self.heading_publisher_ = self.create_publisher(Float64, "/gps/heading", 10)
        # self.snr_publisher_ = self.create_publisher(Float32MultiArray, "/gps/snr", 10)
        self.gnss_data_publisher_ = self.create_publisher(Gnss, "gnss", sensor_qos)

        self.socket = None
        self.latest_azimuth = None  # 最新の方位情報を保持する変数
        self.gsv_sats = {}  # GSVメッセージから衛星情報を一時保存する辞書
        self.latest_snr_msg = Float32MultiArray()

        # 接続とデータ受信を開始
        self.connect_and_read()

    def get_status_from_gga(self, msg):
        """GGAの品質フラグからNavSatStatusを決定する"""
        if msg.gps_qual == 0:
            return NavSatStatus.STATUS_NO_FIX
        elif msg.gps_qual == 1:
            return NavSatStatus.STATUS_FIX
        elif msg.gps_qual == 2:
            return NavSatStatus.STATUS_SBAS_FIX  # DGPS
        elif msg.gps_qual in [4, 5]:
            return NavSatStatus.STATUS_GBAS_FIX  # RTK Fixed / Float
        return NavSatStatus.STATUS_NO_FIX

    def connect_and_read(self):
        try:
            self.get_logger().info(f"Connecting to {self.tcp_ip}:{self.tcp_port}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(1.0)
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info("Connection successful.")

            buffer = ""

            while rclpy.ok():
                raw_data = self.socket.recv(1024).decode("utf-8", errors="ignore")
                if not raw_data:
                    break
                buffer += raw_data

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()

                    if not line:
                        continue

                    try:
                        msg = pynmea2.parse(line)

                        # NMEAメッセージの種別に応じた処理
                        if isinstance(msg, pynmea2.types.talker.HDT):
                            if hasattr(msg, "heading") and msg.heading is not None:
                                self.latest_azimuth = msg.heading

                        elif isinstance(msg, pynmea2.types.talker.VTG):
                            if hasattr(msg, "true_track") and msg.true_track is not None:
                                self.latest_azimuth = msg.true_track

                        elif isinstance(msg, pynmea2.types.talker.RMC):
                            if hasattr(msg, "true_course") and msg.true_course is not None:
                                self.latest_azimuth = msg.true_course

                        elif isinstance(msg, pynmea2.types.talker.GSV):
                            # 分割メッセージの1つ目なら、衛星データを初期化
                            if msg.msg_num == 1:
                                self.gsv_sats.clear()

                            # 1メッセージに最大4つ含まれる衛星情報を辞書に格納
                            for i in range(1, 5):
                                prn = getattr(msg, f"sv_prn_num_{i}")
                                snr = getattr(msg, f"snr_{i}")
                                if prn is not None and snr is not None:
                                    # pynmea2では空のSNRは空文字列になるため、intに変換
                                    try:
                                        self.gsv_sats[msg.talker, prn] = int(
                                            snr
                                        )  # msg.talkerを追加
                                    except (ValueError, TypeError):
                                        pass  # 変換できない場合は無視

                            # 分割メッセージの最後なら、トピックに保存
                            if msg.msg_num == msg.num_messages:
                                # snr_msg = Float32MultiArray()
                                # 辞書から信号強度のリストを作成
                                snr_values = [float(s) for s in self.gsv_sats.values()]
                                self.latest_snr_msg.data = snr_values

                                # self.snr_publisher_.publish(snr_msg)
                                self.get_logger().info(
                                    f"Published satellite SNRs ({len(self.gsv_sats)} sats): {
                                        self.gsv_sats
                                    }"
                                )

                        elif isinstance(msg, pynmea2.types.talker.GGA) and msg.latitude != 0.0:
                            gnss_data_msg = Gnss()
                            now = self.get_clock().now().to_msg()

                            # NavSatFixメッセージの作成と配信
                            fix_msg = NavSatFix()
                            fix_msg.header.stamp = now
                            fix_msg.header.frame_id = "gnss_link"

                            fix_msg.status.status = self.get_status_from_gga(msg)
                            fix_msg.status.service = NavSatStatus.SERVICE_GPS

                            fix_msg.latitude = msg.latitude
                            fix_msg.longitude = msg.longitude

                            try:
                                altitude_val = float(msg.altitude)
                                geo_sep_val = float(msg.geo_sep) if msg.geo_sep else 0.0
                                gnss_data_msg.geoid_altitude = altitude_val
                                gnss_data_msg.geoid_separation = geo_sep_val
                                fix_msg.altitude = altitude_val + geo_sep_val
                            except (ValueError, TypeError):
                                # 型エラー時の処理
                                pass

                            # 共分散の計算
                            try:
                                hdop = float(msg.horizontal_dil)
                                if hdop > 0:
                                    # 通常の処理
                                    h_variance = (self.hdop_error_factor * hdop) ** 2
                                    v_variance = (self.hdop_error_factor * hdop * 1.5) ** 2

                                    fix_msg.position_covariance[0] = h_variance
                                    fix_msg.position_covariance[4] = h_variance
                                    fix_msg.position_covariance[8] = v_variance
                                    fix_msg.position_covariance_type = (
                                        NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                                    )

                                    self.get_logger().info(
                                        f"Publishing: time = {fix_msg.header.stamp.sec}, "
                                        f"qual = {fix_msg.status.status}, lat ={msg.latitude:.6f}, long ={msg.longitude:.6f}, "
                                        f"cov_x = {h_variance:.2f}, cov_y = {h_variance:.2f}, cov_z = {v_variance:.2f}"
                                    )
                                else:
                                    # hdopが0以下だった場合
                                    fix_msg.position_covariance_type = (
                                        NavSatFix.COVARIANCE_TYPE_UNKNOWN
                                    )
                                    self.get_logger().info(
                                        f"Publishing: time ={fix_msg.header.stamp.sec}, "
                                        f"qual ={fix_msg.status.status}, lat = {msg.latitude:.6f}, long = {msg.longitude:.6f}, "
                                    )
                            except (ValueError, TypeError, AttributeError):
                                # hdopがNone, 空文字列, または不正な値だった場合
                                fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                                self.get_logger().info(
                                    f"Publishing: time = {fix_msg.header.stamp.sec}, "
                                    f"qual ={fix_msg.status.status}, lat ={msg.latitude:.6f}, long ={msg.longitude:.6f}, "
                                )

                            # self.fix_publisher_.publish(fix_msg)
                            gnss_data_msg.fix = fix_msg

                            gnss_data_msg.snr = self.latest_snr_msg.data

                            # Float64メッセージ（方位）の作成と配信
                            if self.latest_azimuth is not None:
                                azimuth_msg = Float64()
                                azimuth_msg.data = self.latest_azimuth  # データを直接代入

                                # self.heading_publisher_.publish(azimuth_msg)
                                gnss_data_msg.azimuth = azimuth_msg.data

                                self.get_logger().info(
                                    f"Direction: {self.latest_azimuth:.2f} degrees"
                                )

                            self.gnss_data_publisher_.publish(gnss_data_msg)

                    except pynmea2.ParseError as e:
                        self.get_logger().warning(
                            f'Failed to parse NMEA sentence: "{line}". Reason: {e}'
                        )

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            sys.exit(1)
        finally:
            if self.socket:
                self.socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = GnssPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
