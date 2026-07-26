"""
QEKF(error-state quaternion EKF)が依存するクォータニオン・回転演算の
ユーティリティ関数。

クォータニオンは Hamilton 記法、q = [qw, qx, qy, qz] (shape (4,1)) を採用。
Joan Sola, "Quaternion kinematics for the error-state Kalman filter"
(arXiv:1711.02508) の記法・式番号(QEKF.py内のコメント参照)に対応。
"""

import numpy as np


def cross(v):
    """3次元ベクトル v の外積行列(skew-symmetric matrix)を返す (3x3)"""
    v = np.asarray(v).flatten()
    return np.array([
        [0.0,   -v[2],  v[1]],
        [v[2],   0.0,  -v[0]],
        [-v[1],  v[0],  0.0],
    ])

def quaternion_to_euler(q):
    """クォータニオン[qw,qx,qy,qz] -> (roll, pitch, yaw) [rad], ZYX順"""
    q = np.asarray(q).flatten()
    qw, qx, qy, qz = q
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1.0, 1.0))
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return roll, pitch, yaw

def quaternion_to_rotation_matrix(q):
    """クォータニオン q=[qw,qx,qy,qz] -> 回転行列 R (body -> world, 3x3)"""
    q = np.asarray(q).flatten()
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qz*qw),     2 * (qx*qz + qy*qw)],
        [2 * (qx*qy + qz*qw),     1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qx*qw)],
        [2 * (qx*qz - qy*qw),     2 * (qy*qz + qx*qw),     1 - 2 * (qx**2 + qy**2)],
    ])


def quaternion_product(q1, q2):
    """Hamilton積 q1 ⊗ q2 -> shape (4,1)"""
    q1 = np.asarray(q1).flatten()
    q2 = np.asarray(q2).flatten()
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([[w], [x], [y], [z]])


def rotation_vector_to_quaternion(vector, angle):
    """
    回転軸方向のベクトル vector (非単位ベクトル可) と回転角 angle [rad] から
    クォータニオン(4,1)を生成する(指数写像)。
    QEKF.integrate() では vector=(omega_m - omega_b), angle=norm(vector)*dt
    として呼ばれる想定。
    """
    vector = np.asarray(vector).flatten()
    norm_v = np.linalg.norm(vector)
    if norm_v < 1e-12:
        return np.array([[1.0], [0.0], [0.0], [0.0]])
    axis = vector / norm_v
    qw = np.cos(angle / 2.0)
    qxyz = axis * np.sin(angle / 2.0)
    return np.array([[qw], [qxyz[0]], [qxyz[1]], [qxyz[2]]])


def rodrigues(omega, angle):
    """
    Rodriguesの回転公式。回転軸ベクトル omega と回転角 angle から
    回転行列(3x3)を返す。QEKFの誤差状態ヤコビアン F_x の姿勢ブロック
    (R(ω dt)^T に相当)で使用。
    """
    omega = np.asarray(omega).flatten()
    norm_om = np.linalg.norm(omega)
    if norm_om < 1e-12:
        return np.eye(3)
    axis = omega / norm_om
    K = cross(axis)
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
