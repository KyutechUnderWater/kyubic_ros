# 水中ブイ検出 ROS 2パッケージ

YOLOでブイを検出し、画像中心からの正規化水平偏差、機体正面に対するヨー角、および任意で三次元相対位置を配信します。暫定カメラ条件は800×600です。

## 座標系

`base\_link`はFRD座標として扱います。

```text
X: 機体前方が正
Y: 機体右方向が正
Z: 機体下方向が正
```

## メッセージ

```text
std\_msgs/Header header
bool detected
bool yaw\_valid
bool position\_valid
float32 confidence
float32 normalized\_horizontal\_offset # -1<a<1 右が正　
float32 relative\_buoy\_yaw\_rad # 右が正
float32 relative\_buoy\_x\_m
float32 relative\_buoy\_y\_m
float32 relative\_buoy\_z\_m
```

### normalized\_horizontal\_offset

BBox中心`u`と実入力画像幅`W`から毎フレーム計算します。

```text
image\_center\_u = (W - 1) / 2
normalized\_horizontal\_offset =
    (u - image\_center\_u) / (W / 2)
```

意味は概ね次のとおりです。

```text
-1.0: 画像左端
 0.0: 画像中央
+1.0: 画像右端
```

右方向を正とし、出力は安全のため`\[-1, 1]`へ制限します。実際の入力画像幅から計算するため、焦点距離、主点、ブイまでの距離には依存しません。未検出時はNaNです。

## ヨー角

正面・水平カメラの場合は概ね次です。

```text
yaw = atan2(u - cx, fx)
```

右側を正、左側を負、中央を0とし、単位はradです。現在は暫定内部パラメータを使用するため、中央寄せ制御では`normalized\_horizontal\_offset`を優先し、ヨー角は補助値として扱うことを推奨します。

## 暫定カメラ条件

```yaml
camera:
  provisional: true
  fx: 400.0
  fy: 400.0
  cx: 399.5
  cy: 299.5
  calibration\_width: 800
  calibration\_height: 600
```

水平画角約90度、正方画素を仮定した値であり、実測校正値ではありません。入力画像が別解像度の場合、同一画角という仮定でコードが内部パラメータを拡大縮小します。ただし、クロップやアスペクト比変更がある場合は正確ではありません。

## モデル

ユーザー提供の`best.pt`を同梱しています。

```text
buoy\_detector/models/best.pt
```

## 入出力

```text
入力: /camera/image\_raw
型:   sensor\_msgs/msg/Image

出力: /buoy/relative\_position
型:   buoy\_interfaces/msg/BuoyRelativePosition
```

## 状態

```text
未検出:
  detected=false
  normalized\_horizontal\_offset=NaN
  yaw\_valid=false
  position\_valid=false

検出・中央偏差とヨー角のみ有効:
  detected=true
  normalized\_horizontal\_offset=有限値
  yaw\_valid=true
  position\_valid=false
```

位置推定は暫定校正値による誤制御を避けるため、既定で無効です。

## ビルド

```bash
cd \~/underwater\_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \\
  --packages-select buoy\_interfaces buoy\_detector
source install/setup.bash
```

## 実行

```bash
ros2 launch buoy\_detector buoy\_detector.launch.py \\
  input\_image\_topic:=/camera/image\_raw \\
  output\_topic:=/buoy/relative\_position \\
  body\_frame\_id:=base\_link \\
  position\_estimation\_enabled:=false \\
  device:=cpu
```

## 制御側の推奨使用法

校正前の中央寄せ制御では次を使用します。

```text
detected == true
normalized\_horizontal\_offset < 0: 左側
normalized\_horizontal\_offset > 0: 右側
abs(normalized\_horizontal\_offset)が小さい: 中央付近
```

デッドバンド例は`0.03～0.08`程度から実機調整します。

