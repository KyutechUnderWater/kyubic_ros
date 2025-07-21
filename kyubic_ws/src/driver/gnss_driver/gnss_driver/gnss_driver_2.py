import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import socket
import pynmea2

class GnssDriver(Node):
    def __init__(self):
        super().__init__('gnss_publisher_node')
        # NavSatFixメッセージを配信するPublisherを作成
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Xportの接続情報
        self.tcp_ip = '172.30.51.160'
        self.tcp_port = 5050
        self.socket = None
        
        # 接続とデータ受信を開始
        self.connect_and_read()

    def connect_and_read(self):
        try:
            self.get_logger().info(f'Connecting to {self.tcp_ip}:{self.tcp_port}...')
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info('Connection successful.')
            
            # 受信データを一時的に保持するバッファ
            buffer = ""
            
            while rclpy.ok():
                # データを受信してバッファに追加
                raw_data = self.socket.recv(1024).decode('utf-8', errors='ignore')
                if not raw_data:
                    # 接続が切れた場合
                    break
                buffer += raw_data

                # バッファ内に改行があるか確認
                while '\n' in buffer:
                    # バッファを最初の改行で分割
                    line, buffer = buffer.split('\n', 1)
                    # 前後の空白や\rを削除
                    line = line.strip()
                    
                    # 空白行は無視
                    if not line:
                        continue
                        
                    try:
                        msg = pynmea2.parse(line)
                        if isinstance(msg, pynmea2.types.talker.GGA) and msg.latitude != 0.0:
                            fix_msg = NavSatFix()
                            fix_msg.header.stamp = self.get_clock().now().to_msg()
                            fix_msg.header.frame_id = 'gps_link'
                            fix_msg.status.status = fix_msg.status.STATUS_FIX
                            fix_msg.status.service = fix_msg.status.SERVICE_GPS
                            
                            fix_msg.latitude = msg.latitude
                            fix_msg.longitude = msg.longitude
                            if msg.altitude is not None:
                                fix_msg.altitude = msg.altitude
                            
                            self.publisher_.publish(fix_msg)
                            self.get_logger().info(f'Publishing: lat={msg.latitude}, lon={msg.longitude}')
                    
                    # パースエラーが発生した場合はログに出力（passで無視しない）
                    except pynmea2.ParseError as e:
                        self.get_logger().warning(f'Failed to parse NMEA sentence: "{line}". Reason: {e}')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            if self.socket:
                self.socket.close()
def main(args=None):
    rclpy.init(args=args)
    node = GnssDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
