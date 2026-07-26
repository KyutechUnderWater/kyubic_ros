import numpy as np

# ---------- MAVLink 接続 ----------
MAVLINK_CONNECTION_STRING = 'udpin:192.168.2.1:14550'

# ---------- DVL-75 ($DVEXT) 接続 ----------
# dvl_alllog_csv.py(参考実装)に合わせた値。実際のネットワーク構成と一致しているか要確認。
DVL_BIND_IP = '192.168.2.10'
DVL_UDP_PORT = 27000

# ---------- QEKF 初期パラメータ (実機キャリブレーション後に要チューニング) ----------
# 公称状態 x: [p(3), v(3), q(4), a_bias(3), gyro_bias(3), g(3)] = 19
_X0 = [
    0.0, 0.0, 0.0,          # position (NED) [m]
    0.0, 0.0, 0.0,          # velocity (NED) [m/s]
    1.0, 0.0, 0.0, 0.0,     # quaternion [qw, qx, qy, qz]
    0.0, 0.0, 0.0,          # accel bias
    0.0, 0.0, 0.0,          # gyro bias
    0.0, 0.0, 9.81,         # gravity (NED, z下向き)
]
_DX0 = np.zeros((18, 1))
_P0 = np.eye(18) * 0.1

_STD_A = 0.1               # m/s^2
_STD_GYRO = 0.01           # rad/s
_STD_DVL = 0.02            # m/s
_STD_DEPTH = 0.05          # m
_STD_ORIENTATION = 0.01    # quaternion units
_STD_A_BIAS = 1e-4
_STD_GYRO_BIAS = 1e-5

_DVL_OFFSET = np.zeros(3)
_BAROMETER_OFFSET = np.zeros(3)
_IMU_OFFSET = np.zeros(3)

# ---------- 制御ゲイン・閾値 ----------
POSITION_TOLERANCE = 0.2  # m, 水平方向の到達とみなす距離
DEPTH_TOLERANCE = 0.1     # m, 深度方向の到達とみなす距離

MAX_DT = 0.5  # s, これを超えるdtは発散防止のためクランプ(要チューニング)

PID_YAW_OUTER = dict(
    kp=0.5, ki=0.0, kd=0.0,
    output_limits=(-20.0, 20.0),   # deg/s。初回は控えめな最大旋回速度から
    windup_limit=40.0,             # Ki=0の間は無効。後でIを入れる時のため仮置き
    angle_error_deg=True,          # 誤差を-180~180に正規化(角度なので必須)
)

PID_YAW_INNER = dict(
    kp=20.0, ki=0.0, kd=0.0,
    output_limits=(-500.0, 500.0),  # rコマンド。フルレンジ(±1000)よりまず控えめに
    windup_limit=25.0,
    angle_error_deg=False,          # 誤差は既にdeg/s(角度ではない)ので正規化不要
)

PID_HEAVE_OUTER = dict(
    kp=0.3, ki=0.0, kd=0.0,
    output_limits=(-0.3, 0.3),      # m/s。小型ROVの初回テストとして控えめな昇降速度
    windup_limit=0.6,
    angle_error_deg=False,
)

PID_HEAVE_INNER = dict(
    kp=800.0, ki=0.0, kd=0.0,
    output_limits=(-300.0, 300.0),  # 500中立からのオフセット量(送信時に+500する)
    windup_limit=0.4,
    angle_error_deg=False,
)

PID_SURGE_OUTER = dict(
    kp=0.15, ki=0.0, kd=0.0,
    output_limits=(0.0, 0.3),       # m/s。距離は常に0以上なので下限も0
    windup_limit=0.6,
    angle_error_deg=False,
)

PID_SURGE_INNER = dict(
    kp=1000.0, ki=0.0, kd=0.0,
    output_limits=(0.0, 400.0),     # xコマンド。初回は後退方向を使わない(0始まり)
    windup_limit=0.4,
    angle_error_deg=False,
)

# ---------- ハイドロフォン・画像処理連携 (要チューニング) ----------
BINGER_SEARCH_ASSUMED_DISTANCE = 5.0  # m, ハイドロフォンは距離を測れないための仮定値
HYDROPHONE_PITCH_THRESHOLD_DEG = 30.0  # deg, これを下回ったら「十分近い」とみなす(要検証)
YOLO_CONFIDENCE_THRESHOLD = 0.5       # これ未満の検出は無視する
YOLO_STALE_TIMEOUT = 1.0              # s, timestampがこれより古い検出は無視する

# ---------- ミッションタイミング (要チューニング) ----------
MAIN_LOOP_HZ = 10.0          # main.pyのFSM+制御ループの目標周波数
INIT_WAIT_SEC = 180.0        # 起動後、深度7mへ潜り始めるまでの待機時間 [s]
SEARCH_TIMEOUT_SEC = 420.0   # SEARCH_HYDROPHONEに入ってからのタイムアウト [s]
MIC_CONFIRM_TIMEOUT_SEC = 45.0  # discovered_target確定後、マイクで確認が取れるまでの上限時間 [s]
