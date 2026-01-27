#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from localization_msgs.msg import GlobalPose

import depthai as dai
import time
from pathlib import Path
import threading
import subprocess
import os
import csv
import json


class HeadlessOakCameraNode(Node):
    """
    ROS 2トピックをトリガーとして動作する、ヘッドレス（GUIなし）のOAKカメラノード。
    起動時にハードコードされた固定のカメラパラメータが適用される。
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
        self.capture_still_event = threading.Event()
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

        self.global_pose: GlobalPose = GlobalPose()

        # 6. ROS 2 サブスクライバの作成
        self.photo_sub = self.create_subscription(
            GlobalPose, "/localization/global_pose", self.photo_trigger_callback, 10
        )
        self.video_start_sub = self.create_subscription(
            String, "trigger_video_start", self.video_start_callback, 10
        )
        self.video_stop_sub = self.create_subscription(
            String, "trigger_video_stop", self.video_stop_callback, 10
        )

        # 7. データ処理用の別スレッドを開始
        self.stop_threads_event = threading.Event()
        self.video_thread = threading.Thread(target=self.video_loop)
        self.still_thread = threading.Thread(target=self.still_loop)

        self.video_thread.start()
        self.still_thread.start()

        # 8. CSVファイルの準備
        self.setup_csv_logger()

        self.get_logger().info("ノード準備完了。ROSトピックのトリガーを待っています...")
        self.get_logger().info("終了するには Ctrl+C を押してください。")

    def create_pipeline(self):
        """DepthAIパイプラインを構築する"""
        pipeline = dai.Pipeline()

        # --- ノードの作成 ---
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        video_enc = pipeline.create(dai.node.VideoEncoder)

        # カメラ制御用の入力ノードを追加
        control_in = pipeline.create(dai.node.XLinkIn)
        control_in.setStreamName("control")

        # --- 出力ノード ---
        xout_video_encoded = pipeline.create(dai.node.XLinkOut)
        xout_video_encoded.setStreamName("h265")

        xout_still_uncompressed = pipeline.create(dai.node.XLinkOut)
        xout_still_uncompressed.setStreamName("still")

        # --- カメラ設定 ---
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        cam_rgb.setFps(30)
        cam_rgb.setInterleaved(False)

        # --- エンコーダ設定 ---
        video_enc.setDefaultProfilePreset(
            cam_rgb.getFps(), dai.VideoEncoderProperties.Profile.H265_MAIN
        )

        # --- パイプラインの接続 ---
        cam_rgb.video.link(video_enc.input)
        cam_rgb.video.link(xout_still_uncompressed.input)
        video_enc.bitstream.link(xout_video_encoded.input)
        control_in.out.link(cam_rgb.inputControl)

        return pipeline

    def setup_queues(self):
        """デバイスから入出力キューを取得する"""
        self.h265_queue = self.device.getOutputQueue(name="h265", maxSize=30, blocking=False)
        self.still_queue = self.device.getOutputQueue(name="still", maxSize=4, blocking=False)
        self.control_queue = self.device.getInputQueue("control")

    def apply_initial_camera_settings(self):
        """起動時にカメラの固定設定を適用する"""
        self.get_logger().info("ハードコードされたカメラ設定を適用しています...")

        # --- ここでパラメータを編集 ---
        exp_time = 2000  # 露光時間 (us), 1-33000
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
    def photo_trigger_callback(self, msg):
        self.get_logger().info("静止画トリガー受信 -> 次のフレームを保存します。")
        self.global_pose = msg
        self.capture_still_event.set()

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
            self.get_logger().error(f"一時ファイル {self.h265_filepath} が見つかりません。")
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
                command,
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(f"MP4変換成功: {mp4_filepath}")
            os.remove(self.h265_filepath)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(
                f"FFmpegでの変換に失敗しました。コマンド: '{' '.join(command)}' エラーコード: {e.returncode}"
            )
        except FileNotFoundError:
            self.get_logger().error(
                "`ffmpeg` コマンドが見つかりません。システムにffmpegがインストールされているか確認してください。"
            )

    # --- データ処理ループ (スレッドで実行) ---
    def still_loop(self):
        """静止画撮影の要求を待ち、フレームと位置情報を保存するループ"""
        import cv2

        while not self.stop_threads_event.is_set():
            if self.capture_still_event.wait(timeout=0.5):
                self.capture_still_event.clear()
                in_still = self.still_queue.get()
                if in_still is not None:
                    now = self.get_clock().now()
                    timestamp = (
                        f"{now.nanoseconds // 1_000_000_000}.{now.nanoseconds % 1_000_000_000}"
                    )
                    save_filepath = self.save_dir / f"image{self.image_count}_{timestamp}.jpg"

                    cv2.imwrite(str(save_filepath), in_still.getCvFrame())
                    self.get_logger().info(f"静止画を保存しました: {save_filepath}")

                    self.csv_writer.writerow(
                        [
                            save_filepath.name,
                            self.global_pose.header.stamp.sec,
                            self.global_pose.header.stamp.nanosec,
                            self.global_pose.header.frame_id,
                            self.global_pose.status.depth.id,
                            self.global_pose.status.imu.id,
                            self.global_pose.status.dvl.id,
                            self.global_pose.coordinate_system_id,
                            self.global_pose.ref_pose.latitude,
                            self.global_pose.ref_pose.longitude,
                            self.global_pose.ref_pose.plane_x,
                            self.global_pose.ref_pose.plane_y,
                            self.global_pose.ref_pose.meridian_convergence,
                            self.global_pose.current_pose.latitude,
                            self.global_pose.current_pose.longitude,
                            self.global_pose.current_pose.plane_x,
                            self.global_pose.current_pose.plane_y,
                            self.global_pose.current_pose.meridian_convergence,
                            self.global_pose.azimuth,
                            self.global_pose.depth,
                            self.global_pose.altitude,
                        ]
                    )
                    self.csv_file.flush()
                self.image_count += 1

    def video_loop(self):
        """エンコードされた動画データを一時ファイルに書き込むループ"""
        while not self.stop_threads_event.is_set():
            h265_packet = self.h265_queue.tryGet()
            if h265_packet is not None and self.is_recording and self.h265_file_handle:
                h265_packet.getData().tofile(self.h265_file_handle)
            else:
                time.sleep(0.001)

    def destroy_node(self):
        """ノードのシャットダウン処理"""
        self.get_logger().info("ノードをシャットダウンします...")
        self.stop_threads_event.set()

        if self.is_recording:
            self.video_stop_callback(String())

        self.get_logger().info("Waiting for threads to finish...")
        self.video_thread.join(timeout=2.0)
        self.still_thread.join(timeout=2.0)

        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f"Closed {self.filename}")

        if hasattr(self, "device"):
            self.device.close()
            self.get_logger().info("Device closed.")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeadlessOakCameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"予期せぬエラーが発生しました: {e}", exc_info=True)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
