# 水中ブイ検出 ROS 2パッケージ（base_link FRD版）

YOLOでブイを検出し、BBoxから推定した相対位置を機体座標系`base_link`で配信するROS 2パッケージです。

## 座標系

このシステムでは`base_link`を次のFRD座標として使用します。

```text
X: 機体前方が正
Y: 機体右方向が正
Z: 機体下方向が正
```

> 注意：一般的なROSのREP-103で使われる`base_link`（X前・Y左・Z上）とはY・Zの符号が異なります。同一システム内のTF、制御ノード、可視化設定でもこのFRD定義へ統一してください。

カメラ光学座標は次の向きです。

```text
Xc: 画像右
Yc: 画像下
Zc: カメラ前方
```

正面・水平に取り付けたカメラの既定変換は次のとおりです。

```text
body_x = camera_z
body_y = camera_x
body_z = camera_y
```

つまり、回転行列は以下です。

```text
[0 0 1]
[1 0 0]
[0 1 0]
```

実際のカメラ取付姿勢と位置は`buoy_detector/config/buoy_position.yaml`の`transform`で設定します。

## パッケージ構成

```text
underwater_ws/
└── src/
    ├── buoy_interfaces/
    └── buoy_detector/
```

- `buoy_interfaces`: カスタムメッセージを生成します。
- `buoy_detector`: カメラ画像を購読し、YOLO推論、カメラ座標計算、機体座標変換を行います。

## カスタムメッセージ

```text
std_msgs/Header header
bool detected
bool position_valid
float32 confidence
float32 relative_buoy_x_m
float32 relative_buoy_y_m
float32 relative_buoy_z_m
```

各座標の意味：

```text
relative_buoy_x_m: 機体前後方向。前方が正
relative_buoy_y_m: 機体左右方向。右方向が正
relative_buoy_z_m: 機体上下方向。下方向が正
```

`header.frame_id`の既定値は`base_link`です。座標が無効な場合、X・Y・Zには`NaN`を格納します。

## 設定YAML

```yaml
transform:
  camera_to_body_rotation:
    - [0.0, 0.0, 1.0]
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
  camera_to_body_translation_m: [0.0, 0.0, 0.0]
```

並進ベクトルは、`base_link`原点から見たカメラ光学原点の位置を、`[前、右、下]`の順でメートル指定します。

## 入出力

入力：

```text
/camera/image_raw
sensor_msgs/msg/Image
```

出力：

```text
/buoy/relative_position
buoy_interfaces/msg/BuoyRelativePosition
```

入力画像の`header.stamp`を出力へ引き継ぎ、`header.frame_id`には`body_frame_id`パラメータを設定します。

## ビルド

```bash
cd ~/underwater_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select buoy_interfaces buoy_detector
source install/setup.bash
```

## 実行

```bash
ros2 launch buoy_detector buoy_detector.launch.py \
  input_image_topic:=/camera/image_raw \
  output_topic:=/buoy/relative_position \
  body_frame_id:=base_link \
  device:=cpu
```

水中校正前にYOLO通信だけ確認する場合：

```bash
ros2 launch buoy_detector buoy_detector.launch.py \
  model_path:=/absolute/path/to/best.pt \
  body_frame_id:=base_link \
  position_estimation_enabled:=false
```

## 動作確認

```bash
ros2 interface show buoy_interfaces/msg/BuoyRelativePosition
ros2 topic echo /buoy/relative_position
ros2 topic hz /buoy/relative_position
```
