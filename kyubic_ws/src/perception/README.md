# 水中ブイ検出 ROS 2パッケージ v0.6.0

BlueROVのカメラ画像へYOLO推論を行い、ブイの検出状態、信頼度、画像中心からの水平・垂直正規化偏差、機体正面に対するヨー角、任意の三次元相対位置を配信します。

今回の版では、提供された最新版の座標計算データを参考に、1920×1080のBlue Robotics Low-Light HD USB CameraをBlueROV2のアクリルドーム内で使用する条件へ設定を更新しました。ただし、焦点距離1530は水中屈折を概算した暫定値であり、実測校正値ではありません。

## パッケージ構成

```text
perception/
├── buoy_interfaces/
│   ├── msg/BuoyRelativePosition.msg
│   ├── CMakeLists.txt
│   └── package.xml
└── buoy_detector/
    ├── buoy_detector/
    │   ├── __init__.py
    │   ├── buoy_detector_node.py
    │   └── buoy_geometry.py
    ├── config/buoy_position.yaml
    ├── launch/buoy_detector.launch.py
    ├── models/best.pt
    ├── package.xml
    ├── setup.cfg
    └── setup.py
```

`perception`は整理用ディレクトリであり、ROSパッケージ名は`buoy_interfaces`と`buoy_detector`のままです。

## 入出力

```text
入力トピック: /driver/blue_rov/camera/image_raw
入力型:       sensor_msgs/msg/Image
入力QoS:      qos_profile_sensor_data

出力トピック: /buoy/relative_position
出力型:       buoy_interfaces/msg/BuoyRelativePosition
出力QoS:      depth 10
```

## カスタムメッセージ

```text
std_msgs/Header header

bool detected
bool yaw_valid
bool position_valid

float32 confidence
float32 normalized_horizontal_offset
float32 normalized_vertical_offset
float32 relative_buoy_yaw_rad
float32 relative_buoy_x_m
float32 relative_buoy_y_m
float32 relative_buoy_z_m
```

### フラグ

```text
detected:
  YOLOが指定クラスのBBoxを出力した場合にtrue

yaw_valid:
  relative_buoy_yaw_radを使用できる場合にtrue

position_valid:
  relative_buoy_x_m / y_m / z_mを使用できる場合にtrue
```

`detected=true`でも、BBoxが画像端へ接触した場合や距離計算が許容範囲外の場合は、`position_valid=false`になります。

## 正規化オフセット

### 水平

```text
image_center_u = (W - 1) / 2
normalized_horizontal_offset =
    (bbox_center_u - image_center_u) / (W / 2)
```

```text
-1付近: 画像左端
 0:      画像中央
+1付近: 画像右端
```

右方向を正とします。

### 垂直

```text
image_center_v = (H - 1) / 2
normalized_vertical_offset =
    (bbox_center_v - image_center_v) / (H / 2)
```

```text
-1付近: 画像上端
 0:      画像中央
+1付近: 画像下端
```

下方向を正とします。

水平・垂直とも、実入力画像の幅と高さから毎フレーム計算し、`[-1, 1]`へ制限します。焦点距離、主点、対象距離には依存しません。

## ヨー角

カメラ光学座標の視線ベクトルを機体座標へ回転し、FRD座標の水平面で計算します。

```text
yaw = atan2(body_ray_y, body_ray_x)
```

```text
負: ブイが左側
 0: ブイが機体正面
正: ブイが右側
```

単位はradです。

## 座標系

このパッケージでは`base_link`をFRDとして扱います。

```text
X: 機体前方が正
Y: 機体右方向が正
Z: 機体下方向が正
```

カメラ光学座標は次です。

```text
Xc: 画像右
Yc: 画像下
Zc: カメラ前方
```

既定の回転行列は次の対応です。

```text
body_x = camera_z
body_y = camera_x
body_z = camera_y
```

## カメラ設定

```yaml
camera:
  provisional: true
  fx: 1530.0
  fy: 1530.0
  cx: 960.0
  cy: 540.0
  calibration_width: 1920
  calibration_height: 1080
```

この値は、空気中の想定焦点距離約1144へ、水中屈折による約1.33倍を適用した概算値です。実際の水中カメラ校正後に置き換えてください。

入力解像度が異なる場合は、同一画角を仮定して内部パラメータを拡大縮小します。クロップ、デジタルズーム、アスペクト比変更がある場合は、ヨー角と三次元位置の精度が低下します。

## 距離推定

既定方式は`pinhole_area`です。

```text
Zc = sqrt(
  fx × fy × buoy_width_m × buoy_height_m
  / bbox_area
)
```

設定値は次です。

```yaml
range:
  model: pinhole_area
  buoy_width_m: 0.110
  buoy_height_m: 0.170
  min_range_m: 0.10
  max_range_m: 10.00
```

BBoxが画像端から2 pixel以内へ接触した場合、面積が欠けるため位置座標を無効化します。ただし、検出状態、信頼度、正規化偏差、ヨー角は可能な限り配信します。

## 位置推定の既定状態

`position_estimation_enabled`は既定で`false`です。

```text
position_estimation_enabled=false:
  detected、confidence、水平・垂直偏差、ヨー角を配信
  position_valid=false
  X/Y/Z=NaN

position_estimation_enabled=true:
  BBox、距離範囲、画像端判定が有効な場合だけX/Y/Zを配信
```

水中校正と実距離試験が完了するまでは、`false`のまま使用してください。

## ビルド

ワークスペース例:

```text
kyubic_ws/src/perception/
├── buoy_interfaces/
└── buoy_detector/
```

```bash
cd ~/kyubic_ws
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y

colcon build \
  --symlink-install \
  --packages-select \
  buoy_interfaces \
  buoy_detector

source install/setup.bash
```

## 単体起動

```bash
ros2 launch buoy_detector buoy_detector.launch.py
```

位置推定を有効にする場合:

```bash
ros2 launch buoy_detector buoy_detector.launch.py \
  position_estimation_enabled:=true
```

## BlueROV起動チェーンへの組み込み

`buoy_detector`の実運用起動は、共通`behavior_tree.launch.py`ではなく、BlueROV側の`bluerov_bt.launch.py`またはBlueROV bringupへ含めます。

```python
buoy_detector_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [
                FindPackageShare("buoy_detector"),
                "launch",
                "buoy_detector.launch.py",
            ]
        )
    ),
    launch_arguments={
        "input_image_topic":
            "/driver/blue_rov/camera/image_raw",
        "output_topic":
            "/buoy/relative_position",
        "body_frame_id":
            "base_link",
        "confidence":
            "0.25",
        "device":
            "cpu",
        "position_estimation_enabled":
            "false",
    }.items(),
)
```

`bluerov_control_bt_nodes/package.xml`には、用途に応じて次を追加します。

```xml
<depend>buoy_interfaces</depend>
<exec_depend>buoy_detector</exec_depend>
```

`buoy_interfaces`はC++のBT葉ノードがカスタム型を使用するため、`buoy_detector`はBlueROV側launchから検出ノードを起動するためです。

## Subscriber側の基本判定

```cpp
if (!msg->detected) {
  return;
}

if (msg->confidence < 0.40F) {
  return;
}

if (!std::isfinite(msg->normalized_horizontal_offset) ||
    !std::isfinite(msg->normalized_vertical_offset)) {
  return;
}
```

三次元位置は次の条件をすべて満たす場合だけ使用します。

```cpp
if (
  msg->position_valid &&
  std::isfinite(msg->relative_buoy_x_m) &&
  std::isfinite(msg->relative_buoy_y_m) &&
  std::isfinite(msg->relative_buoy_z_m)
) {
  // 位置座標を使用
}
```

## 確認コマンド

```bash
ros2 topic type /driver/blue_rov/camera/image_raw
ros2 topic hz /driver/blue_rov/camera/image_raw
ros2 topic info /driver/blue_rov/camera/image_raw --verbose
```

```bash
ros2 interface show buoy_interfaces/msg/BuoyRelativePosition
ros2 topic type /buoy/relative_position
ros2 topic echo /buoy/relative_position
ros2 topic info /buoy/relative_position --verbose
```

期待する通信関係:

```text
CameraDriver
  /driver/blue_rov/camera/image_raw
        ↓
buoy_detector
  /buoy/relative_position
        ↓
CheckBuoyPosition
CheckBuoyInFrame
WriteVisionWaypointCSV
```

## 今回の更新点

- カメラ基準を800×600の仮定値から1920×1080の水中暫定値へ更新
- `fx`、`fy`を1530へ更新
- 最大位置推定距離を10 mへ更新
- BBoxの非有限値、幅、高さ、面積の検証を強化
- `pinhole_area`内でも距離範囲を検証
- `inverse_scale`でゼロスケールを拒否
- 画像端接触時は位置だけを無効化
- 水平・垂直正規化偏差とヨー角の既存出力を維持
- カスタムメッセージの`detected`と`position_valid`を分離したまま維持
- BlueROVカメラトピックを既定入力へ設定
