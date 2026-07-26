import sys

from pymavlink import mavutil

from params import MAVLINK_CONNECTION_STRING

GAIN = 100  # GainUp/GainDown用。Sample/BlueROV2 with Python demo.pyの初期値に合わせる


# Start a connection listening on a UDP port
BLUEROV = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING)
# Wait for the first heartbeat
heartbeat = BLUEROV.recv_match(type='HEARTBEAT', condition=f'HEARTBEAT.get_srcComponent() == {mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1}', blocking=True)
print(heartbeat)


def Arm():
    # master.arducopter_arm() or:
    BLUEROV.mav.command_long_send(
        BLUEROV.target_system,
        BLUEROV.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    BLUEROV.motors_armed_wait()
    print('Armed!')


def Disarm():
    # master.arducopter_disarm() or:
    BLUEROV.mav.command_long_send(
        BLUEROV.target_system,
        BLUEROV.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    # wait until disarming confirmed
    print("Waiting for the vehicle to disarm")
    BLUEROV.motors_disarmed_wait()
    print('Disarmed!')


def ChangeMode(mode='MANUAL'):
    """
    :param mode: 'MANUAL' or 'STABILIZE' or 'ALT_HOLD'
    """
    if mode not in BLUEROV.mode_mapping():
        print("Unknown mode : {}".format(mode))
        print('Try: ', list(BLUEROV.mode_mapping().keys()))
        sys.exit(1)
    mode_id = BLUEROV.mode_mapping()[mode]
    BLUEROV.mav.set_mode_send(
        BLUEROV.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)


def SetPwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    # https://www.ardusub.com/developers/rc-input-and-output.html
    if channel_id < 1 or 14 < channel_id:
        print("Channel does not exist.")
        return
    BLUEROV.mav.command_long_send(
        BLUEROV.target_system, BLUEROV.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel_id,
        pwm,
        0, 0, 0, 0, 0
    )


def ManualControl(x, y, z, yaw):
    """
    MAVLink MANUAL_CONTROL を送信する。
    x: 前後方向速度 (-1000〜1000, 正=前進)
    y: 左右方向速度 (-1000〜1000, 正=右)
    z: 上下方向スロットル (0〜1000, 500が中立。値が小さいほど沈む方向、大きいほど浮上方向)
    yaw: 旋回レート (-1000〜1000)
    """
    BLUEROV.mav.manual_control_send(
        BLUEROV.target_system,
        x,
        y,
        z,
        yaw,
        0)


def CameraTilt(tilt, roll=0, pan=0):
    """
    Moves gimbal to given position
    Args:
        tilt (float): tilt angle in centidegrees (0 is forward)
        roll (float, optional): pan angle in centidegrees (0 is forward)
        pan  (float, optional): pan angle in centidegrees (0 is forward)
    """
    BLUEROV.mav.command_long_send(
        BLUEROV.target_system,
        BLUEROV.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        1,
        tilt,
        roll,
        pan,
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)


# TODO: GAIN は初期化されないままのグローバル変数として参照されている既知バグ。
# 元の BlueRovStabilityControl.py から挙動を変えずそのまま移設。
def GainUp():
    global GAIN
    GAIN += 100
    if 1000 < GAIN:
        GAIN = 1000
    print("Gain is", GAIN)


def GainDown():
    global GAIN
    GAIN -= 100
    if GAIN < 0:
        GAIN = 0
    print("Gain is", GAIN)
