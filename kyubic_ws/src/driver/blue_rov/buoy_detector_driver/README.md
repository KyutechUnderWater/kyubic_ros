# buoy_detector_driver

`perception/buoy_detector`(画像処理班のYOLOブイ検出、`buoy_interfaces/msg/BuoyRelativePosition`
をpublish)と、`bluerov_control_bt_nodes`の`CheckBuoyDetected`(`bluerov_control_msgs/msg/
BuoyDetection`を購読)をつなぐ**中継アダプタノード**。座標変換は一切行わず、メッセージ型の変換と
検出品質判定だけを行う。

対象読者: ROS2に触れ始めたばかりの開発者。

## 1. なぜこのノードが必要か

`CheckBuoyDetected`は`BuoyDetection`(`bluerov_control_msgs`)を購読する設計だったが、この
メッセージ型へ実データをpublishするノードは存在しなかった(`BuoyDetection.msg`自体のコメントにも
「実装なし(未実装)。実装時はこのメッセージを実データで発行するノードに差し替える」と明記)。

一方、画像処理班が共有した`buoy_detector_node`(`perception/buoy_detector`)は、YOLO検出と
カメラ座標→機体座標(FRD、base_link)の変換までを既に行い、`BuoyRelativePosition`
(`buoy_interfaces`)としてpublishする。**この2つのメッセージ型は座標系・単位は同じだが、
型が違うので直接は繋がらない。** このノードはその変換だけを行う薄いアダプタ。

**なぜdriver層(`driver/blue_rov/`)に置くか**: 5層パイプライン(Driver→機体非依存の共通topic→
Localization→Planner→Output)のうち、「センサーの生データを共通形式のtopicへ変換して出す」という
役割は、`dvl75_driver`/`imu_driver`/`mavlink_driver`と同じ位置づけとして扱う。ただし
`BuoyDetection.msg`は`control/automatic/bluerov_control_msgs`(control層)に属するため、
このパッケージはdriver層でありながらcontrol層のメッセージパッケージへ依存する形になる
(5層パイプラインとしては逆方向の依存)。これは依頼者との事前合意による配置であり、
座標変換ロジックそのものは持たない薄いアダプタという性質上、許容している。

## 2. 変換ロジック

`BuoyRelativePosition`の`detected`/`position_valid`の2段階を、`dvl75_driver`等が使う
`common_msgs::msg::Status`(NORMAL/WARNING/ERROR)と同じ考え方で判定してから`BuoyDetection`の
`detected`フラグへ反映する(`BuoyDetection.msg`自体にStatusフィールドを追加するのではなく、
判定はこのノード内部だけで完結させる)。

| `BuoyRelativePosition`の状態 | 内部判定 | `BuoyDetection.detected` |
|---|---|---|
| `detected=false` | ERROR(何も見えていない) | `false` |
| `detected=true`, `position_valid=false` | ERROR(検出はしたが座標は信用できない) | `false` |
| `detected=true`, `position_valid=true` | NORMAL | `true` |

WARNINGは使わない(実質NORMAL/ERRORの2値判定)。

**ERROR判定の周期は、直前の有効値を保持せず、正直に`detected=false`として報告する。**
これはPart A(`bluerov_control_bt_nodes/README.md`§9、DVL/depth/IMUのERROR時に直前値を保持する
ように修正したバグ)とは意図的に異なる方針。理由: `CheckBuoyDetected`は`RetryUntilSuccessful`
ループの中で毎フレーム再評価されるため、検出が一瞬途切れても次のフレームでまた試せばよい。
一方、odomの位置推定は連続的な積分値であり欠落を埋める必要があるため、両者は文脈が異なる。
`detected=false`のとき、`relative_buoy_x_m`/`y_m`/`z_m`は`NaN`にする
(`BuoyRelativePosition.msg`自身の「`position_valid=false`の場合はNaN」という規約と揃える。
`CheckBuoyDetected`は`detected=false`の間はこれらの値を読まないため実害は無いが、値の意味を
明確にするため)。

`confidence`と`header`(画像撮影時刻)はそのまま引き継ぐ。座標変換(`buoy_geometry.py`の
`estimate_range_pinhole_area`/`transform_camera_to_body`等)は既に`buoy_detector_node`側で
完結しているため、このノードでは行わない。

## 3. 使い方

```bash
ros2 launch buoy_detector_driver buoy_detector_driver.launch.py
```

`input_topic`(既定`/buoy/relative_position`)、`output_topic`(既定`/perception/buoy_detection`、
`bluerov_bt.launch.py`が前提とするremap先と一致)をlaunch引数で上書きできる。

### 単体テスト

`perception/buoy_detector`側のYOLOモデル・カメラ校正が未整備でも、このノード自体は
`ros2 topic pub`で単体テストできる(§4参照)。

```bash
ros2 launch buoy_detector_driver buoy_detector_driver.launch.py

# 別ターミナル
ros2 topic pub /buoy/relative_position buoy_interfaces/msg/BuoyRelativePosition \
  "{detected: true, position_valid: true, confidence: 0.8, relative_buoy_x_m: 1.0, relative_buoy_y_m: 0.0, relative_buoy_z_m: 0.5}" --rate 5

# 別ターミナルで確認: detected=true、座標がそのまま伝わる
ros2 topic echo /perception/buoy_detection
```

`detected: false`または`position_valid: false`を送ると、`detected=false`かつ座標がNaNになる
ことを確認できる。

## 4. 既知の制約(実機テスト前に確認すること)

- **`perception/buoy_detector`はまだ実行できない**: `models/`にYOLOモデル実体
  (`best_ncnn_model/`または`best.pt`)が未配置(`README.txt`のみ)。加えて
  `config/buoy_position.yaml`の`camera.fx`/`fy`がプレースホルダーの`0.0`のままで、
  `position_estimation_enabled=true`で起動すると`CameraIntrinsics.validate()`が例外を
  送出する。モデルの入手と水中カメラ校正は画像処理班側の別タスク
- **`relative_buoy_z_m`のNED符号一致は実機未検証**: `check_buoy_detected.cpp`のコメント通り、
  `relative_buoy_z_m`の「正=下方向」というNED慣習との符号一致は、画像処理担当者とすり合わせて
  実機で確認すること。ここがズレるとROVが上下逆に動く
- このアダプタ自体は`WrenchPlan`等を一切扱わない(購読→変換→publishのみ)。機体を制御する経路には
  入らない
- `bluerov_bt.launch.py`への組み込み(本番起動チェーンへの追加)は今回のスコープ外。モデル・
  カメラ校正が揃ってから別途判断する
