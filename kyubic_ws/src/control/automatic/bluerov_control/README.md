# bluerov_control

BlueROVのミッション制御(状態遷移/FSM)ROS2ノード。リポジトリルートの `blueRovControl/`
(非ROS2の単体Pythonスクリプト群)を、`kyubic_ws` の既存ROS2資産(`mavlink_driver`,
`dvl75_driver`, `localization`, `emergency`)を使う形に移植したもの。

対象読者: ROS2に触れ始めたばかりの開発者。このREADMEはROS2の用語を、このパッケージの
実コードに紐づけて説明する。

## 1. 起動方法

以下を **この順番で** 起動する(それぞれ別ターミナル、コンテナ内で実行):

```bash
# 1. BlueROVのハードウェアドライバ(MAVLink接続 + DVL-75)
ros2 launch driver_launcher blue_rov_driver.launch.py

# 2. センサ融合(位置・姿勢推定)。/localization/odom を発行する
ros2 launch localization localization_components.launch.py

# 3. このパッケージ(ミッション制御 + 緊急浮上ノード)
ros2 launch bluerov_control bluerov_control.launch.py
```

3が起動すると、`bluerov_control_node` は `INIT` 状態から自動的にミッションを開始する
(起動直後に `set_armed` サービスで自動武装を試みる)。停止するには `Ctrl+C`(全ノードとも
シャットダウン時に中立のWrench = 推力ゼロを送ってから終了する)。

## 2. このコードとROS2の概念

初心者向けに、ROS2の主要概念をこのパッケージの実際のコードに紐づけて説明する。

- **Node(ノード)**: 1つの実行単位。このパッケージには `bluerov_control_node`
  ([node.py](bluerov_control/node.py)の`BlueRovControlNode`クラス)という1つのノードしかない。
  ミッションの17状態すべてを、1つのノードの中の1つのタイマーコールバック
  (`_tick()`)が10Hz(`main_loop_hz`パラメータ)で処理する。なぜ状態ごとにノードを
  分けなかったかは、状態間で位置・目標座標などの情報を大量に共有する必要があり、
  分割すると逆にトピック/サービス経由のやり取りが増えて複雑になるため
  (詳しくはこのタスクの探索フェーズの議論を参照)。

- **Topic(トピック)/ Publisher・Subscriber**: ノード間でデータを流す仕組み。
  このノードは以下をSubscribe(受信)する:
  - `odom` (実際には `/localization/odom` にremap): 現在の位置・姿勢・速度
  - `vehicle_state`: BlueROVの武装/接続状態
  - `hydrophone_bearing` / `buoy_detection`: センサースタブ(§4参照)

  以下をPublish(送信)する:
  - `robot_force` (`/driver/blue_rov/mavlink_driver/robot_force` にremap):
    推力指令(`geometry_msgs/WrenchStamped`)。これが実際にROVを動かす唯一の出力。
  - `heartbeat`: `mavlink_driver`の「デッドマン監視」用信号。0.2秒ごとに送り続けないと、
    `mavlink_driver`側が安全のため推力コマンドを無視するようになる。

- **Remapping(リマッピング)**: ノードのコード自体は `odom` や `robot_force` という
  短い相対名しか知らない。実際のトピック名(`/localization/odom`等)への対応付けは
  [launch/bluerov_control.launch.py](launch/bluerov_control.launch.py)の`remappings=[...]`
  で行う。これにより`node.py`は「BlueROV用」と決め打ちせず、将来他機体や別名前空間でも
  launchファイルの変更だけで再利用できる。

- **Parameter(パラメータ)**: 実行時に調整できる値。[config/bluerov_control.param.yaml](config/bluerov_control.param.yaml)
  にまとめてあり、`self.declare_parameter(...)` (node.py)で宣言、`ros2 param get/set`
  コマンドで実行中にも確認・変更できる(ただしこのノードは起動時に一度読むだけで、
  変更を動的に反映するコールバックは実装していないので、値を変えたい場合はノードの
  再起動が必要)。

- **Launch(起動ファイル)**: 複数ノードをまとめて、パラメータ・remappingを添えて起動する
  設定ファイル。[launch/bluerov_control.launch.py](launch/bluerov_control.launch.py)は
  `bluerov_control_node`に加えて、緊急浮上用の`emergency_surfacing`ノード(既存の
  `emergency`パッケージのノードを、BlueROV向けにremapして再利用)も一緒に起動する。

- **Service(サービス)**: 1回のリクエスト/レスポンス型の呼び出し。`set_armed`
  (`mavlink_driver`への武装リクエスト)と、`emergency_surfacing`の
  `change_state`(緊急浮上の起動/停止)で使っている。トピックと違い「呼んだらすぐ
  1回だけ答えが返る」もので、`node.py`では`call_async()` + コールバックで非同期に
  呼んでいる(タイマーコールバックの中で応答をブロック待ちすると、他の処理が
  止まってしまうため)。

- **Lifecycle Node(ライフサイクルノード)**: `emergency_surfacing`はこの仕組みで
  作られている(`unconfigured → inactive → active`という状態を持つ特殊なノード)。
  `bluerov_control_node`はEMERGENCY状態に入った瞬間だけ、`change_state`サービスで
  `configure`→`activate`を呼び、`emergency_surfacing`を有効化する(有効化されると
  一定の上向き推力を出し続ける)。

## 3. パラメータのチューニング方法

`config/bluerov_control.param.yaml`の各値の意味はファイル内のコメントを参照。特に重要なもの:

- `pid.*` (6系統): 位置制御(outer)と速度制御(inner)の2段カスケードPID。
  **旧`blueRovControl`のゲイン初期値をそのまま引き継いでいるが、出力先が変わったため
  数値の意味も変わっている**。旧実装はMAVLinkの`MANUAL_CONTROL`(-1000〜1000、
  PWM相当スケール)に直接値を送っていたが、このノードは`WrenchStamped`を
  `mavlink_driver`の`robot_force`に送り、`mavlink_driver`側の`axis.<name>.limit`
  パラメータ(例: `forward`は52.0)で物理量として解釈されPWMに変換される。
  そのため**実機での再チューニングが必須**。目安: `pid.surge_inner.output_limits`等の
  値を小さく始め、`mavlink_driver`の`axis.forward.limit`(既定52.0)との組み合わせで
  実際の前進速度を見ながら徐々に上げる。
- `descend_depth_m` / `dive_offset_m` / `post_check_rise_offset_m` / `surface_rise_offset_m`:
  ミッションの深度形状。プール/競技フィールドの水深に合わせて調整。
- `hydrophone_pitch_threshold_deg`: 符号の向きが実機で未検証(コード内コメント参照)。
  逆方向に動くようであれば、`flow.py`の`handle_search_hydrophone`内の不等号を
  `<`に変更する必要がある。

## 4. スタブ(未実装)一覧

以下は実センサ/実処理が未接続で、トピックのインターフェースだけを定義した状態:

| トピック | 型 | 説明 | 実装時にすべきこと |
|---|---|---|---|
| `hydrophone_bearing` | `bluerov_control_msgs/HydrophoneBearing` | ハイドロフォンの測角(pitch/yaw) | 実際のハイドロフォン読み出しノードを作り、このトピックに発行する |
| `buoy_detection` | `bluerov_control_msgs/BuoyDetection` | 画像処理によるブイ検出 | 画像処理担当のノードが、このトピックに発行する |

未受信の間、`bluerov_control_node`はクラッシュせず安全側(その場で待機/検出なし扱い)に
フォールバックする。動作確認には`ros2 topic pub`でダミー値を注入できる:

```bash
ros2 topic pub /perception/hydrophone_bearing bluerov_control_msgs/msg/HydrophoneBearing \
  "{pitch_deg: 35.0, yaw_deg: 0.0}" --rate 5

ros2 topic pub /perception/buoy_detection bluerov_control_msgs/msg/BuoyDetection \
  "{detected: true, relative_buoy_x_m: 1.0, relative_buoy_y_m: 0.0, relative_buoy_z_m: 0.5, confidence: 0.9}" --rate 5
```

また、`debug.image_processing_available` / `debug.mic_data_from_above` パラメータを
`ros2 param set`(起動前ならYAML編集)で切り替えることで、VISUAL系/HYDRO系どちらの
分岐にも人為的に進めてテストできる。

## 5. 既知の制約(移植に伴うトレードオフ)

- 姿勢/速度/深度の推定を、旧実装の自前QEKF(BNO08x IMU + DVL自前パース)から
  既存の`localization`パッケージ(`/localization/odom`)に置き換えた。この結果:
  - 深度は`mavlink_driver`が`VFR_HUD.alt`から算出する値になり、旧実装で未実装だった
    `get_depth_data()`のクラッシュ問題は解消された。
  - 姿勢はArduSub内蔵IMU由来になり、旧実装が使っていたBNO08x(ESP32経由)とは別センサ・
    別精度特性になる。
  - yaw角速度はQEKFが推定していたジャイロバイアス補正が失われ、生値をそのまま使っている
    (`control.py`のdocstring参照)。ドリフトが問題になる場合は`localization`側の改善が必要。
- 深度(`z_depth`)・ハイドロフォンpitchの符号が、NED慣習(正=下方向)と実機で一致しているかは
  **未検証**(旧実装から引き継いだ制約。コード内コメント参照)。

## 6. 旧Python実装(`blueRovControl/`)のレビュー

移植作業を通じて判明した、旧実装自体の実装漏れ・改善余地:

- `function.get_depth_data()`が無条件に`NotImplementedError`を投げ、毎tick呼ばれる
  `state_estimation()`から呼び出されるため、**旧実装は実行すると初回tickで確実に
  クラッシュする**状態だった(バロメータ未接続だったため)。
- 安全機構は`EmergencyProblem`(深度<0.1mでEMERGENCY)ただ1つ。リーク検知・電圧監視・
  通信断watchdog・ジオフェンスが無い。
- `DvlStream`/`ImuStream`のソケットに再接続ロジックが無く、切断時はIMU側は例外
  (`RuntimeError`、main.pyでは未捕捉)、DVL側は`None`を返す(こちらは呼び出し側で
  グレースフルに扱われる)という非対称な挙動だった。
- `mavlink_io.py`の`Arm()`/`Disarm()`/`ChangeMode()`/`SetPwm()`/`CameraTilt()`/
  `GainUp()`/`GainDown()`はすべて定義済みだが、ミッションフローのどこからも
  呼ばれていなかった(武装は外部(QGroundControl等)での事前実施が前提)。
- `common.normalize_deg`と`PID._normalize`が同じ±180度正規化を別々に実装しており、
  重複していた(移植版では`PID._normalize`のみ残し、`normalize_deg`は削除)。
- ハイドロフォンpitch閾値の不等号の向き(`flow.py`)が実機未検証のままコメントで
  明記されていた。
- `perception.body_offset_to_ned`のz符号・座標系変換も、実機(画像処理・ハイドロフォン
  担当)との実際の仕様すり合わせが必要と明記されていた。
