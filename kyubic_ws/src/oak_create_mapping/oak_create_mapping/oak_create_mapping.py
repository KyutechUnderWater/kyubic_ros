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
import subprocess
import os

class OakCameraSubscriber(Node):
    """
    ROS 2トピックをトリガーにして、OAKカメラで12MPの静止画と、
    ハードウェアエンコードを利用した12MP/30FPSの高画質動画を最適化して撮影・保存するノード。
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
        self.h265_file_handle = None
        self.h265_filepath = None
        # 静止画撮影要求をスレッドセーフに伝えるためのイベント
        self.capture_still_event = threading.Event()
        self.get_logger().info(f"データ保存先フォルダ: {self.save_dir.resolve()}")

        # 3. DepthAI パイプラインの構築
        self.pipeline = dai.Pipeline()

        # --- ノードの作成 ---
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        video_enc = self.pipeline.create(dai.node.VideoEncoder)
        xout_video_encoded = self.pipeline.create(dai.node.XLinkOut)
        xout_still_uncompressed = self.pipeline.create(dai.node.XLinkOut)

        xout_video_encoded.setStreamName("h265")
        xout_still_uncompressed.setStreamName("still")

        # --- カメラ設定 (12MP, 30FPS) ---
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        cam_rgb.setFps(30)
        cam_rgb.setInterleaved(False)

        # --- エンコーダ設定 (H.265) ---
        video_enc.setDefaultProfilePreset(cam_rgb.getFps(), dai.VideoEncoderProperties.Profile.H265_MAIN)

        # --- パイプラインの接続 ---
        # 動画ストリームをエンコーダと静止画用の出口の両方に接続
        cam_rgb.video.link(video_enc.input)
        cam_rgb.video.link(xout_still_uncompressed.input)
        video_enc.bitstream.link(xout_video_encoded.input)

        # 4. デバイスの初期化とキューの取得
        try:
            self.device = dai.Device(self.pipeline)
            self.h265_queue = self.device.getOutputQueue(name="h265", maxSize=30, blocking=False)
            self.still_queue = self.device.getOutputQueue(name="still", maxSize=4, blocking=False)
            self.get_logger().info("OAK-1カメラの準備ができました。")
        except Exception as e:
            self.get_logger().error(f"OAK-1カメラの初期化に失敗: {e}")
            raise

        # 5. ROS 2 サブスクライバの作成
        self.photo_sub = self.create_subscription(String, 'trigger_photo', self.photo_trigger_callback, 10)
        self.video_start_sub = self.create_subscription(String, 'trigger_video_start', self.video_start_callback, 10)
        self.video_stop_sub = self.create_subscription(String, 'trigger_video_stop', self.video_stop_callback, 10)
        
        # 6. データ処理用の別スレッドを開始
        self.stop_threads_event = threading.Event()
        self.video_thread = threading.Thread(target=self.video_loop)
        self.still_thread = threading.Thread(target=self.still_loop)
        self.video_thread.start()
        self.still_thread.start()

        self.get_logger().info("ノード準備完了。最適化されたコード。静止画・動画のトリガーを待っています...")

    # --- コールバック関数群 ---
    def photo_trigger_callback(self, msg):
        self.get_logger().info(f'静止画トリガー受信: "{msg.data}" -> 次のフレームを保存します。')
        self.capture_still_event.set() # イベントをセットして静止画保存スレッドに通知

    def video_start_callback(self, msg):
        if self.is_recording:
            self.get_logger().warn("すでに録画中です。")
            return
        
        self.is_recording = True
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.h265_filepath = self.save_dir / f"video_{timestamp}.h265"
        self.h265_file_handle = open(self.h265_filepath, 'wb')
        self.get_logger().info(f'動画撮影を開始しました。一時ファイル: {self.h265_filepath}')

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
        """H.265ファイルをMP4に変換する"""
        if not self.h265_filepath or not self.h265_filepath.exists():
            self.get_logger().error(f"一時ファイル {self.h265_filepath} が見つかりません。")
            return

        mp4_filepath = self.h265_filepath.with_suffix(".mp4")
        command = ["ffmpeg", "-framerate", "30", "-i", str(self.h265_filepath), "-c", "copy", "-y", str(mp4_filepath)]
        
        try:
            self.get_logger().info(f"変換中: {self.h265_filepath} -> {mp4_filepath}")
            subprocess.run(command, check=True, capture_output=True, text=True)
            self.get_logger().info(f"変換成功。一時ファイル {self.h265_filepath} を削除します。")
            os.remove(self.h265_filepath)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"FFmpegでの変換に失敗しました。エラー: {e.stderr}")
        except FileNotFoundError:
            self.get_logger().error("`ffmpeg` コマンドが見つかりません。システムにffmpegがインストールされているか確認してください。")

    # --- データ処理ループ (スレッドで実行) ---
    def still_loop(self):
        """静止画撮影の要求を待ち、フレームを保存するループ"""
        while not self.stop_threads_event.is_set():
            # イベントがセットされるのを待つ (タイムアウト付き)
            if self.capture_still_event.wait(timeout=0.5):
                # イベントをクリアして次の要求に備える
                self.capture_still_event.clear()
                
                # キューから最新のフレームを取得
                in_still = self.still_queue.get() 
                if in_still is not None:
                    save_filepath = self.save_dir / f"image{self.image_count}.jpg"
                    still_frame = in_still.getCvFrame()
                    cv2.imwrite(str(save_filepath), still_frame)
                    self.get_logger().info(f"静止画を保存しました: {save_filepath}")
                    self.image_count += 1

    def video_loop(self):
        """エンコードされた動画データを一時ファイルに書き込むループ"""
        while not self.stop_threads_event.is_set():
            h265_packet = self.h265_queue.tryGet()
            if h265_packet is not None and self.is_recording and self.h265_file_handle:
                h265_packet.getData().tofile(self.h265_file_handle)
            else:
                time.sleep(0.001) # CPU負荷を軽減

    def destroy_node(self):
        self.get_logger().info("ノードをシャットダウンします...")
        if self.is_recording:
            self.video_stop_callback(String())
        
        self.stop_threads_event.set()
        self.video_thread.join(timeout=1.0)
        self.still_thread.join(timeout=1.0)

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