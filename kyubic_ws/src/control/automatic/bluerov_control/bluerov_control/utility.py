"""姿勢(Euler角)にまつわる回転演算のユーティリティ関数。

blueRovControl/utility_functions.py からの移植だが、姿勢の入力元が
blueRovControl独自のQEKF(クォータニオン出力)から /localization/odom
(localization_msgs/Pose.orientation、roll/pitch/yawのEuler角 [deg]) に
変わったため、クォータニオンではなくEuler角を直接受け取る形に書き換えている。
qekf.py が使っていた quaternion_product/rotation_vector_to_quaternion/rodrigues/cross は
QEKF専用だったため、QEKFを使わないこの移植では不要(移植していない)。

NED慣習(z正=下方向)を前提にしたbody->world回転行列。符号の向きは旧実装と
同じく実機で要検証(README「未検証の符号」参照)。
"""

import math

import numpy as np


def rotation_matrix_from_euler_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """roll/pitch/yaw [deg] (ZYX順、NED慣習) -> 回転行列 R (body -> world, 3x3)"""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
