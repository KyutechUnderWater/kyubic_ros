#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import depthai as dai
import time
from pathlib import Path
import threading

class OakCameraSubscriber(Node):
    """
    ROS 2トピックをトリガーにして、OAKカメラで高解像度の静止画と動画を撮影・保存するノード。
    - /trigger_photo: 静止画を1枚撮影
    - /trigger_video_start: 動画撮影を開始
    - /trigger_video_stop: 動画撮影を停止
    """
    def __init__(self):
        # 1. ROS 2 ノードの初期化
        super().__init__('oak_camera_subscriber_node')

        # 2. 保存設定と状態管理
        run_timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.save_dir = Path(f"DATA_{run_timestamp}")
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.image_count = 1
        self.is_recording = False
        self.video_writer = None
        self.get_logger().info(f"データ保存先フォルダ: {self.save_dir.resolve()}")

        # 3. DepthAI パイプラインの構築
        self.pipeline = dai.Pipeline()

        # --- カメラノードの共通設定 ---
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        
        # --- 静止画用の設定 (12MP) ---
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        control_in = self.pipeline.create(dai.node.XLinkIn)
        control_in.setStreamName("control")
        xout_still = self.pipeline.create(dai.node.XLinkOut)
        xout_still.setStreamName("still")
        control_in.out.link(cam_rgb.inputControl)
        cam_rgb.still.link(xout_still.input)

        # --- 動画用の設定 (1080p) ---
        cam_rgb.setVideoSize(1920, 1080)
        cam_rgb.setFps(30)
        xout_video = self.pipeline.create(dai.node.XLinkOut)
        xout_video.setStreamName("video")
        cam_rgb.video.link(xout_video.input)

        # 4. デバイスの初期化とキューの取得
        try:
            self.device = dai.Device(self.pipeline)
            self.control_queue = self.device.getInputQueue(name="control")
            self.still_queue = self.device.getOutputQueue(name="still", maxSize=4, blocking=False)
            self.video_queue = self.device.getOutputQueue(name="video", maxSize=30, blocking=False)
            self.get_logger().info("OAK-1カメラの準備ができました。")
        except Exception as e:
            self.get_logger().error(f"OAK-1カメラの初期化に失敗: {e}")
            raise

        # 5. ROS 2 サブスクライバの作成
        self.photo_sub = self.create_subscription(String, 'trigger_photo', self.photo_trigger_callback, 10)
        self.video_start_sub = self.create_subscription(String, 'trigger_video_start', self.video_start_callback, 10)
        self.video_stop_sub = self.create_subscription(String, 'trigger_video_stop', self.video_stop_callback, 10)
        
        # 6. データ処理用の別スレッドを開始
        self.stop_event = threading.Event()
        self.saver_thread = threading.Thread(target=self.save_loop)
        self.video_thread = threading.Thread(target=self.video_loop)
        self.saver_thread.start()
        self.video_thread.start()

        self.get_logger().info("ノード準備完了。静止画・動画のトリガーを待っています...")

    # --- コールバック関数群 ---
    def photo_trigger_callback(self, msg):
        self.get_logger().info(f'静止画トリガー受信: "{msg.data}" -> 撮影を命令します。')
        ctrl = dai.CameraControl()
        ctrl.setCaptureStill(True)
        self.control_queue.send(ctrl)

    def video_start_callback(self, msg):
        if self.is_recording:
            self.get_logger().warn("すでに録画中です。")
            return
        
        self.is_recording = True
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        video_filepath = self.save_dir / f"video_{timestamp}.mp4"
        
        # OpenCVのVideoWriterを初期化
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 30.0
        resolution = (1920, 1080)
        self.video_writer = cv2.VideoWriter(str(video_filepath), fourcc, fps, resolution)
        
        self.get_logger().info(f'動画撮影を開始しました。保存先: {video_filepath}')

    def video_stop_callback(self, msg):
        if not self.is_recording:
            self.get_logger().warn("録画中ではありません。")
            return
            
        self.is_recording = False
        # VideoWriterを解放することでファイルが正しく閉じられる
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        self.get_logger().info("動画撮影を停止し、ファイルを保存しました。")

    # --- データ処理ループ (スレッドで実行) ---
    def save_loop(self):
        """静止画を保存するループ"""
        while not self.stop_event.is_set():
            in_still = self.still_queue.tryGet()
            if in_still is not None:
                save_filepath = self.save_dir / f"image{self.image_count}.jpg"
                still_frame = in_still.getCvFrame()
                cv2.imwrite(str(save_filepath), still_frame)
                self.get_logger().info(f"静止画を保存しました: {save_filepath}")
                self.image_count += 1
            else:
                time.sleep(0.01)

    def video_loop(self):
        """動画フレームを処理するループ"""
        while not self.stop_event.is_set():
            in_video = self.video_queue.tryGet()
            if in_video is not None:
                frame = in_video.getCvFrame()
                if self.is_recording and self.video_writer is not None:
                    self.video_writer.write(frame)
            else:
                time.sleep(0.001)

    def destroy_node(self):
        self.get_logger().info("ノードをシャットダウンします...")
        if self.is_recording:
            self.video_stop_callback(String()) # 録画中なら停止処理を呼ぶ
        self.stop_event.set()
        self.saver_thread.join()
        self.video_thread.join()
        if hasattr(self, 'device'):
            self.device.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OakCameraSubscriber()
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        if node:
            node.get_logger().error(f"エラーまたは中断が発生しました: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
