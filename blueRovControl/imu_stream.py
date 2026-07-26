"""
Adafruit BNO08x搭載ESP32(IMU_CALIB_AND_REC.ino)からのIMUストリームを受信するモジュール。

プロトコル概要(IMU_CALIB_AND_REC.ino準拠):
    - ESP32がこのPCへTCP接続してくる(このモジュールはサーバとして待ち受ける)
    - 接続確立後、"MEASURE" コマンドを送るとESP32がCSVデータ行の送信を開始する
    - データ行フォーマット(1行、カンマ区切り13フィールド、末尾\n):
        Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Qx,Qy,Qz,Qw
      - Ax,Ay,Az : 加速度 [m/s^2] (SH2_ACCELEROMETER、キャリブレーション済み、重力込み=比力)
      - Gx,Gy,Gz : 角速度 [rad/s] (SH2_GYROSCOPE_CALIBRATED)
      - Mx,My,Mz : 磁場 [uT] (今回は未使用)
      - Qx,Qy,Qz,Qw: 姿勢クォータニオン(SH2_ROTATION_VECTOR)。
        ★ w成分が最後にくる順序なので、QEKFが使う[qw,qx,qy,qz]の順に並べ替えて保持する。
    - ESP32側の送信は SH2_ROTATION_VECTOR イベント発火時にのみ行われるため、
      姿勢推定が何らかの理由で止まるとIMUデータ全体(加速度・角速度含む)が
      途絶する点に注意(ファームウェア側の構造)。

IMU_logger.py との違い:
    - CSVファイルへの保存ではなく、直近値をスレッドセーフに保持し get_latest() で
      取得できるようにしている(制御ループから毎周期ポーリングする用途)。
    - 接続確立後、GET_ONCE(精度確認)を経由せず即座に "MEASURE" を送る
      (handleCommandLine() 上、MEASURE単独で計測開始するため、リアルタイム制御用
      には不要な往復を省いている)。
"""

import socket
import threading
import time

import numpy as np


class ImuStream:
    def __init__(self, listen_ip="0.0.0.0", listen_port=5007, stale_timeout=0.5):
        """
        listen_ip, listen_port : ESP32からの接続を待ち受けるアドレス
        stale_timeout          : 直近の受信からこの秒数以上経過していたら
                                  「データ無し」扱いにする [s]
        """
        self._listen_ip = listen_ip
        self._listen_port = listen_port
        self._stale_timeout = stale_timeout

        self._lock = threading.Lock()
        self._latest = None       # dict または None
        self._latest_time = 0.0

        self._server = None
        self._accept_thread = None
        self._running = False

    def start(self):
        """バックグラウンドでサーバを起動し、ESP32の接続を待つ"""
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self._listen_ip, self._listen_port))
        self._server.listen(1)
        self._running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def stop(self):
        self._running = False
        if self._server is not None:
            self._server.close()

    def _accept_loop(self):
        while self._running:
            print("[ImuStream] Waiting for ESP32 connection...")
            try:
                conn, addr = self._server.accept()
            except OSError:
                break  # stop() でソケットが閉じられた場合
            self._handle_connection(conn, addr)

    def _handle_connection(self, conn, addr):
        print(f"[ImuStream] Connected: {addr}")
        conn.settimeout(1.0)
        try:
            conn.sendall(b"MEASURE\n")  # 接続直後に即計測開始を指示
        except Exception:
            conn.close()
            return

        buffer = ""
        while self._running:
            try:
                data = conn.recv(4096)
                if not data:
                    break
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    self._parse_and_store(line.strip())
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[ImuStream] recv error: {e}")
                break
        conn.close()
        print(f"[ImuStream] Disconnected: {addr}")

    def _parse_and_store(self, line):
        # "HELLO_FROM_ESP" / "MODE=..." / ">> MEASURE" / "ACCURACY_ONCE:..." 等の
        # 非データ行は、カンマ13個の数値行にならないため自然に弾かれる。
        if not line or "," not in line:
            return
        parts = line.split(",")
        if len(parts) != 13:
            return
        try:
            values = [float(p) for p in parts]
        except ValueError:
            return

        ax, ay, az, gx, gy, gz, mx, my, mz, qx, qy, qz, qw = values
        with self._lock:
            self._latest = {
                "accel": np.array([ax, ay, az]),     # [m/s^2], body, 比力
                "gyro": np.array([gx, gy, gz]),       # [rad/s], body
                "mag": np.array([mx, my, mz]),        # [uT], body (未使用)
                "quat": np.array([qw, qx, qy, qz]),   # [qw,qx,qy,qz] に並べ替え済み
            }
            self._latest_time = time.time()

    def get_latest(self):
        """
        直近のIMU値をdictで返す('accel','gyro','mag','quat'をキーに持つ)。
        stale_timeoutを超えて更新が無い場合はNone。
        """
        with self._lock:
            if self._latest is None:
                return None
            if time.time() - self._latest_time > self._stale_timeout:
                return None
            return self._latest