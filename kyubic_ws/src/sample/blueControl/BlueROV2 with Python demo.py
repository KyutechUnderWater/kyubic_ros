from pymavlink import mavutil
import sys
import time
import keyboard


GAIN = 100
# Start a connection listening on a UDP port
BLUEROV2 = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
# Wait for the first heartbeat
heartbeat = BLUEROV2.recv_match(type='HEARTBEAT', condition=f'HEARTBEAT.get_srcComponent() == {mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1}', blocking=True)
print(heartbeat)


def Arm():
    # master.arducopter_arm() or:
    BLUEROV2.mav.command_long_send(
        BLUEROV2.target_system,
        BLUEROV2.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    BLUEROV2.motors_armed_wait()
    print('Armed!')


def Disarm():
    # master.arducopter_disarm() or:
    BLUEROV2.mav.command_long_send(
        BLUEROV2.target_system,
        BLUEROV2.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    # wait until disarming confirmed
    print("Waiting for the vehicle to disarm")
    BLUEROV2.motors_disarmed_wait()
    print('Disarmed!')


def ChangeMode(mode='MANUAL'):
    """
    :param mode: 'MANUAL' or 'STABILIZE' or 'ALT_HOLD'
    """
    if mode not in BLUEROV2.mode_mapping():
        print("Unknown mode : {}".format(mode))
        print('Try: ', list(BLUEROV2.mode_mapping().keys()))
        sys.exit(1)
    mode_id = BLUEROV2.mode_mapping()[mode]
    BLUEROV2.mav.set_mode_send(
        BLUEROV2.target_system,
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
    BLUEROV2.mav.command_long_send(
        BLUEROV2.target_system, BLUEROV2.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel_id,
        pwm,
        0, 0, 0, 0, 0
    )


def ManualControl(x, y, z, yaw):
    BLUEROV2.mav.manual_control_send(
        BLUEROV2.target_system,
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
    BLUEROV2.mav.command_long_send(
        BLUEROV2.target_system,
        BLUEROV2.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        1,
        tilt,
        roll,
        pan,
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)


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


# フライトモード変更
ChangeMode("MANUAL")
# 制御権取得
Arm()
# カメラチルト
CameraTilt(45*100)
time.sleep(0.5)
CameraTilt(-45*100)
time.sleep(0.5)
CameraTilt(0*100)
time.sleep(0.5)
# LED点灯
for brightness in range(1100, 1900, 10):
    SetPwm(7, brightness)
    time.sleep(0.01)
for brightness in range(1900, 1100, -10):
    SetPwm(7, brightness)
    time.sleep(0.01)
# グリッパー開閉
SetPwm(11, 1900)
time.sleep(2)
SetPwm(11, 1100)
time.sleep(2)
# 前進
ManualControl(200, 0, 500, 0)
time.sleep(1)
# 後進
ManualControl(-200, 0, 500, 0)
time.sleep(1)
# 浮上
ManualControl(0, 0, 700, 0)
time.sleep(1)
# 潜水
ManualControl(0, 0, 300, 0)
time.sleep(1)
# 停止
ManualControl(0, 0, 500, 0)

# キーボード操作モード
print("Keyboard control mode")
print("Click here!")
print("To abort, press ctrl+c.")
keyboard.on_press_key("page up", lambda _: GainUp())
keyboard.on_press_key("page down", lambda _: GainDown())
while True:
    try:
        x_throttle = 0
        y_throttle = 0
        z_throttle = 500
        yaw_throttle = 0
        if keyboard.is_pressed("w"):
            x_throttle += GAIN
        if keyboard.is_pressed("s"):
            x_throttle -= GAIN
        if keyboard.is_pressed("d"):
            y_throttle += GAIN
        if keyboard.is_pressed("a"):
            y_throttle -= GAIN
        if keyboard.is_pressed("up"):
            z_throttle += GAIN//2
        if keyboard.is_pressed("down"):
            z_throttle -= GAIN//2
        if keyboard.is_pressed("right"):
            yaw_throttle += GAIN
        if keyboard.is_pressed("left"):
            yaw_throttle -= GAIN
        ManualControl(x_throttle, y_throttle, z_throttle, yaw_throttle)
    except KeyboardInterrupt:
        print("Abort!")
        # 停止
        ManualControl(0, 0, 500, 0)
        # 制御権開放
        Disarm()
        time.sleep(1)
        # ボードリセット
        BLUEROV2.reboot_autopilot()
        time.sleep(1)
        break
