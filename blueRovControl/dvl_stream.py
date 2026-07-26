"""
Cerulean Sonar DVL-75からの $DVEXT UDPストリームを受信するモジュール。

dvl_alllog_csv.py(参考実装)のソケット処理を踏襲しつつ、imu_stream.pyの
ImuStreamと同じ設計(バックグラウンドスレッド + get_latest())に揃えている。

このモジュールは生の$DVEXT文字列を保持するだけで、パース自体は
function.parse_dvext()側で行う(パースロジックの置き場所を1箇所にまとめるため)。

$DVPDL(位置・角度の差分メッセージ)は今回は扱わない。必要になったら
_recv_loop()内のフィルタ条件を緩めて、別途保持フィールドを増やす。
"""
import socket
import threading
import time


class DvlStream:
    def __init__(self, bind_ip="192.168.2.10", udp_port=27000, stale_timeout=0.5):
        """
        bind_ip, udp_port : DVLからのUDPパケットを待ち受けるアドレス
                             (dvl_alllog_csv.pyのBIND_IP/UDP_PORTに合わせる。
                              実際のネットワーク構成に応じてparams.pyで要確認)
        stale_timeout      : 直近の受信からこの秒数以上経過していたら
                              「データ無し」扱いにする [s]
        """
        self._bind_ip = bind_ip
        self._udp_port = udp_port
        self._stale_timeout = stale_timeout

        self._lock = threading.Lock()
        self._latest_sentence = None  # 生の$DVEXT文字列
        self._latest_time = 0.0

        self._sock = None
        self._recv_thread = None
        self._running = False

    def start(self):
        """バックグラウンドでUDP受信を開始する"""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._bind_ip, self._udp_port))
        self._sock.settimeout(1.0)
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def stop(self):
        self._running = False
        if self._sock is not None:
            self._sock.close()

    def _recv_loop(self):
        while self._running:
            try:
                payload, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break  # stop()でソケットが閉じられた場合

            text = payload.decode("ascii", errors="ignore").strip()
            if not text.startswith("$DVEXT"):
                continue  # $DVPDLなど、今は使わないメッセージは無視

            with self._lock:
                self._latest_sentence = text
                self._latest_time = time.time()

    def get_latest_sentence(self):
        """
        直近の$DVEXT生文字列を返す(パースはfunction.parse_dvext()側で行う)。
        stale_timeoutを超えて更新が無い場合はNone。
        """
        with self._lock:
            if self._latest_sentence is None:
                return None
            if time.time() - self._latest_time > self._stale_timeout:
                return None
            return self._latest_sentence
