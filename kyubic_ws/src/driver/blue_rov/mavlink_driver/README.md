# mavlink_driver

BlueROV/ArduSubへのMAVLink接続を単独で所有するROS 2ドライバー。RC overrideによる操作、MAVLink telemetryの共通メッセージ化、ARM/DISARMサービスを提供。

## 実装時・今後の開発での参照先

| 参照先 | 実装・改修時の用途 |
| --- | --- |
| [MAVLink Common Message Set](https://mavlink.io/en/messages/common.html) | `HEARTBEAT`、`ATTITUDE`、`BATTERY_STATUS`、`GPS_RAW_INT`、`VFR_HUD`、`SCALED_PRESSURE2`、`COMMAND_ACK`、`RC_CHANNELS_OVERRIDE`のフィールド・単位 |
| [MAVLink Command Protocol](https://mavlink.io/en/services/command.html) | `MAV_CMD_COMPONENT_ARM_DISARM`送信と`COMMAND_ACK`確認の仕様 |
| [ArduSub Pilot Control](https://ardupilot.org/sub/docs/pilot-control.html) | ArduSubにおけるMAVLink RC overrideによる操縦入力の位置付け |
| [ArduSub Auxiliary Functions](https://ardupilot.org/sub/docs/common-auxiliary-functions.html) | RCチャンネルとカメラ・補助機能の割当確認 |
| [ArduSub MAVLink Messages](https://ardupilot.org/sub/docs/ArduSub_MAVLink_Messages.html) | ArduSubが処理・送信するMAVLinkメッセージの確認 |
| [mavlink_driver.py](mavlink_driver/mavlink_driver.py) | ROS/MAVLink変換、接続監視、ARM確認処理の正本 |
| [conversions.py](mavlink_driver/conversions.py) | Wrench・カメラ角度からPWMへの変換の正本 |
| [mavlink_driver.param.yaml](config/mavlink_driver.param.yaml) | 実機用RCチャンネル、PWM範囲、接続先の設定値 |

ArduSubファームウェア、BlueOS、RCチャンネル設定を更新した場合は、上記の公式資料と実機パラメータを照合し、拘束状態でPWM出力を確認すること。

## 起動

```bash
cd <workspace>
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to mavlink_driver --symlink-install
source install/setup.bash
ros2 launch mavlink_driver mavlink_driver.launch.py
```

`launch/mavlink_driver.launch.py`は`/driver/blue_rov/mavlink_driver/mavlink_driver`を起動し、`config/mavlink_driver.param.yaml`をロード。`log_level`引数あり。IMUと深度は機体に依存しない共通Topicの`/driver/imu`と`/driver/depth`へPublishする。

## 通信

| 種別 | 相対名 | 型 | QoS・処理 |
| --- | --- | --- | --- |
| Subscribe | `robot_force` | `geometry_msgs/msg/WrenchStamped` | KeepLast 10。RC軸指令へ変換 |
| Subscribe | `led` | `driver_msgs/msg/LED` | KeepLast 10。AUX PWMへ変換 |
| Subscribe | `camera_tilt` | `driver_msgs/msg/Int32Stamped` | KeepLast 10。カメラPWMへ変換 |
| Subscribe | `heartbeat` | `std_msgs/msg/Bool` | KeepLast 10。制御有効監視 |
| Publish | `depth` | `driver_msgs/msg/Depth` | KeepLast 10。MAVLink圧力計測由来 |
| Publish | `imu` | `driver_msgs/msg/IMU` | KeepLast 10。姿勢は度へ変換 |
| Publish | `power_state` | `driver_msgs/msg/PowerState` | KeepLast 10 |
| Publish | `gnss` | `driver_msgs/msg/Gnss` | KeepLast 10 |
| Publish | `vehicle_state` | `driver_msgs/msg/VehicleState` | KeepLast 10。0.2秒周期 |
| Service | `set_armed` | `std_srvs/srv/SetBool` | `true`: ARM、`false`: DISARM。ACKとheartbeatで確認 |

namespaceは`/driver/blue_rov/mavlink_driver`。Actionは使用なし。RC overrideは`command_rate_hz`周期、GCS heartbeatは1秒周期。

launchファイルは相対名`imu`と`depth`だけを共通Topicへremapする。その他のTopicとServiceは`/driver/blue_rov/mavlink_driver`配下に配置される。

### MAVLink変換と接続状態

- `MAVLINK20=1`を`pymavlink` import前に設定
- `MAV_TYPE_SUBMARINE`の`HEARTBEAT`だけを接続対象として受理
- 最初の対象heartbeat受信時に`MAV_DATA_STREAM_ALL`を10Hzで要求
- 対象system ID以外のtelemetryを破棄
- heartbeatが`connection_timeout_s`を超えて欠落した場合、接続を切断して`reconnect_delay_s`後に再接続

| MAVLink受信メッセージ | ROS 2への変換 |
| --- | --- |
| `HEARTBEAT` | 接続状態、armed、custom mode、system statusを`vehicle_state`へ反映 |
| `BATTERY_STATUS` | 電圧[mV]をV、電流[cA]をAへ変換して`power_state`へPublish |
| `VFR_HUD` | `alt * depth_scale`を`depth.depth`へPublish |
| `SCALED_PRESSURE2` | 温度[centi-degree]を°Cへ変換し、後続`depth.temperature`へ使用 |
| `ATTITUDE` | roll/pitch/yaw[rad]を`imu.orient`のdegreeへ、角速度をdegree/sへ変換して`imu.gyro`へPublish |
| `GPS_RAW_INT` | 緯度・経度を`1e7`、高度を`1000`で除算して`gnss`へPublish |
| `COMMAND_ACK` | ARM/DISARMコマンドの受理確認へ使用 |

`set_armed`は`MAV_CMD_COMPONENT_ARM_DISARM`を`COMMAND_LONG`で送り、対応する`COMMAND_ACK`とvehicle heartbeatのarmed状態が一致した場合だけ成功を返す。自動ARMは実装しない。

### RC override

`robot_force`は`force.x/y/z`をforward/lateral/heave、`torque.x/z`をroll/yawへ対応付ける。各値は`axis.<name>.limit`で正規化し、`axis.<name>.invert`を反映してPWMへ変換。デフォルトの軸channelはroll=2、heave=3、yaw=4、forward=5、lateral=6。

`require_control_heartbeat=true`では、`heartbeat`と`robot_force`が各timeout内にあり、かつvehicleがarmedの場合だけ有効なRC overrideを送信。それ以外では中立またはrelease値を送信する。

## 設定

設定は`config/mavlink_driver.param.yaml`。起動時のみ読込む実装。

| 区分 | パラメータ |
| --- | --- |
| 接続 | `endpoint`、`source_system`、`source_component`、`connection_timeout_s`、`reconnect_delay_s` |
| 制御安全 | `command_rate_hz`、`command_timeout_s`、`control_heartbeat_timeout_s`、`require_control_heartbeat`、`set_armed_timeout_s` |
| RC PWM | `pwm_min`、`pwm_neutral`、`pwm_max`、`axis.<roll|heave|yaw|forward|lateral>.(channel|limit|invert)` |
| カメラ・LED | `camera_*`、`led_left_channel`、`led_right_channel` |
| 計測 | `depth_scale` |

各axisのchannelは1–18、重複不可。cameraとaxisのchannel重複も不正。PWM範囲は`pwm_min <= pwm_neutral <= pwm_max`が必須。機体を拘束した状態で軸方向・PWM範囲を確認すること。

## 確認・停止・制約

```bash
ros2 node info /driver/blue_rov/mavlink_driver/mavlink_driver
ros2 topic echo /driver/imu
ros2 topic echo /driver/depth
ros2 topic echo /driver/blue_rov/mavlink_driver/vehicle_state
ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool "{data: false}"
```

停止は`Ctrl-C`。終了処理で中立RC overrideを送信。`KyutechUnderWaterBlue/blue.py`など、同じ機体へRC overrideを送る別クライアントとの同時実行は不可。
