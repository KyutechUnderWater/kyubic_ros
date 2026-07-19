#!/usr/bin/env python3
"""
jetson_urchin_tracker.py
Jetson Orin NX上で動作するウニ検出および3D座標算出ROS 2ノード (Jazzy対応)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import message_filters
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO

class UrchinTrackerNode(Node):
    def __init__(self):
        super().__init__('urchin_tracker_node')
        
        # パラメータ設定（Jetsonの環境に合わせて変更してください）
        self.declare_parameter('yolo_model', 'yolov8n.pt') # 学習済みのウニ検知モデルのパス
        self.declare_parameter('confidence_threshold', 0.5)
        
        yolo_model_path = self.get_parameter('yolo_model').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        
        # YOLOの初期化
        self.get_logger().info(f"Loading YOLO model: {yolo_model_path}")
        self.model = YOLO(yolo_model_path)
        
        self.bridge = CvBridge()
        
        # Camera Info を一度だけ取得するためのサブスクライバ
        self.camera_info = None
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info', # ZEDのカメラ情報トピック
            self.camera_info_callback,
            10
        )
        
        # RGB画像とDepthマップの同期サブスクライバ
        rgb_topic = '/zed/zed_node/rgb/image_rect_color'
        depth_topic = '/zed/zed_node/depth/depth_registered'
        
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        
        # タイムスタンプが近いもの同士を同期してコールバックを呼ぶ (ズレ許容度: 0.1秒)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)
        
        # ロボットPCへ目標座標をPublishするトピック
        self.target_pub = self.create_publisher(PointStamped, '/urchin_target_position', 10)
        self.get_logger().info("Urchin Tracker Node is ready and waiting for camera images...")

    def camera_info_callback(self, msg):
        """ZEDカメラのレンズ歪み・焦点距離(内部パラメータ)を取得"""
        # K行列 = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        self.camera_info = msg
        self.get_logger().info(f"Camera Info Received! fx:{self.fx:.2f}, fy:{self.fy:.2f}, cx:{self.cx:.2f}, cy:{self.cy:.2f}")
        # 一度取得したら不要なので解除
        self.destroy_subscription(self.cam_info_sub)

    def sync_callback(self, rgb_msg, depth_msg):
        """RGB画像とDepth画像がセットで到着したときに呼ばれる"""
        if self.camera_info is None:
            return # Camera Infoがまだ来ていない場合は計算できないためスキップ
            
        # 1. ROSメッセージをOpenCVの画像（NumPy配列）に変換
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        # Depthは通常 32FC1 (32bit floatで1ピクセル＝距離(メートル))
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        
        # 2. YOLOで「左カメラのRGB画像だけ」を使ってウニを推論
        # verbose=Falseで推論の標準出力をオフにして動作を軽くする
        results = self.model(cv_rgb, verbose=False)
        
        best_box = None
        max_conf = 0.0
        
        # 検出された物体の中で、一番信頼度(Confidence)が高いものを追従対象とする
        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                # ※ ウニ専用モデルの場合、クラスIDは0になることが多いです。必要に応じて条件を追加してください。
                if conf > self.conf_thresh and conf > max_conf:
                    max_conf = conf
                    best_box = box.xyxy[0].cpu().numpy() # [x1, y1, x2, y2]
                    
        # 3. ウニが見つかった場合、3D座標を計算
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            
            # バウンディングボックスの中心ピクセル座標 (u, v) を計算
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)
            
            # 範囲外アクセス防止
            h, w = cv_depth.shape
            u = max(0, min(u, w - 1))
            v = max(0, min(v, h - 1))
            
            # 4. 「全く同じ座標(u,v)」のDepth（距離）を取得する
            z_dist = float(cv_depth[v, u])
            
            # NaN(計測不可)や負の値、0の場合は無効なDepthデータとして弾く
            if np.isnan(z_dist) or np.isinf(z_dist) or z_dist <= 0:
                self.get_logger().warn("Urchin detected, but depth is invalid at the center.")
                return
                
            # 5. ピクセル座標(u, v)と距離(z_dist)から、カメラ基準の3D空間の相対座標(X, Y, Z)を計算
            # 光学カメラ座標系：前方がZ、右がX、下がY
            x_3d = (u - self.cx) * z_dist / self.fx
            y_3d = (v - self.cy) * z_dist / self.fy
            
            # 6. 結果をPointStampedとしてPublish
            point_msg = PointStamped()
            point_msg.header.stamp = rgb_msg.header.stamp
            point_msg.header.frame_id = rgb_msg.header.frame_id # "zed_left_camera_optical_frame" など
            
            point_msg.point.x = x_3d
            point_msg.point.y = y_3d
            point_msg.point.z = z_dist
            
            self.target_pub.publish(point_msg)
            self.get_logger().info(f"Target Published! X:{x_3d:.2f}m, Y:{y_3d:.2f}m, Depth(Z):{z_dist:.2f}m")

            # === 実験用の画面表示（描画） ===
            cv2.rectangle(cv_rgb, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(cv_rgb, (int(u), int(v)), 5, (0, 0, 255), -1)
            text = f"Dist: {z_dist:.2f}m"
            cv2.putText(cv_rgb, text, (int(x1), max(int(y1) - 10, 0)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        # ウニがいない時もカメラ映像を確認できるように画面に表示する
        cv2.imshow("Jetson Urchin Tracker - Camera View", cv_rgb)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = UrchinTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
