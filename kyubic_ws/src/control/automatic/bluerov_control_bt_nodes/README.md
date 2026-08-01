# bluerov_control_bt_nodes

BlueROVをKyubicと同じ **Behavior Tree(BT) + `wrench_planner`(P-PID) + `zero_order_hold`(ZOH) +
`emergency`** 構成で動かすための、BlueROV固有の追加分。

**現状(2026年)**: BlueROVのミッション制御は`behavior_tree`パッケージの
[bt_xml/iwakuni2026.xml](../../../behavior_tree/bt_xml/iwakuni2026.xml)(`BlueROV2Auto`ツリー)
に一本化されている。詳細は§10を参照。

**この文書の構成について(履歴)**: §1〜8は、iwakuni2026.xmlへ統合される前の
「Phase 1/Phase 2」設計(`bt_xml/bluerov_phase1.xml`/`bluerov_phase2.xml`、旧`flow.py`の
17状態を丸ごとBT化する状態遷移方式)についての設計記録。**Phase 1/Phase 2のXMLファイル自体は
削除済みで、現在は実行できない。** ただし、そこで得られた知見・修正の多くは今も現行コードに
生きている(特に§9のPart A/B/C、DVL/odomゼロ値伝播バグの修正は`dvl75_driver`等の現行実装に
そのまま残っている)ため、経緯を記録として残してある。現行の構成・使い方は§10を参照すること。

対象読者: ROS2に触れ始めたばかりの開発者。

## 1. なぜこの構成にしたか(履歴、§10も参照)

旧`bluerov_control`(`kyubic_ws/src/control/automatic/bluerov_control/`)は状態遷移(`flow.py`)と
カスケードPID(`control.py`)が1つのPythonノードに同居する自前実装だった。Kyubicの`main`ブランチは
同じ役割を4つの独立ノードに分離しており、**実機で実績がある**。調査の結果、
`behavior_tree`/`wrench_planner`/`zero_order_hold`/`emergency`はいずれもKyubic固有ハードウェアへの
依存が無く(`package.xml`にドライバ系パッケージへの依存が無い)、`emergency`は既に今の
`bluerov_control`がソース変更なしで再利用できている。この根拠から、**Kyubicの資産をソース変更なしで
再利用し、BlueROV固有の新規部分だけをこのパッケージ(C++)とBT XMLで書く**方針にした。

旧`bluerov_control`(`flow.py`/`control.py`/`node.py`)は**削除していない**。Phase 2完了時点でも
新旧2つの起動経路が並存する(下記2参照)。17状態すべてが移行できたら`control.py`(自前PID)を
正式に廃止する予定。

## 2. 起動方法(履歴。現行の起動手順は§10.4参照)

Phase 2時点の起動手順は以下だった(`bluerov_bt.launch.py`は現在`iwakuni2026.xml`を
起動するよう更新済みなので、このコマンド自体は今もほぼそのまま使える。§10.4参照)。

```bash
# 1. BlueROVのハードウェアドライバ
ros2 launch driver_launcher blue_rov_driver.launch.py

# 2. センサ融合(位置・姿勢推定)
ros2 launch localization localization_components.launch.py

# 3. BT構成、通しでMainTreeを実行(このパッケージ)
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py

# 【旧構成、切り戻し用】従来のFSM+PID(3とは同時に起動しないこと。両方がrobot_forceを
# 送ろうとして競合する)
# ros2 launch bluerov_control bluerov_control.launch.py
```

Phase 2固有の個別SubTreeテスト(`main_tree:=MainTree_VisualPath`等)は、そのSubTree自体が
`bluerov_phase2.xml`の削除とともに無くなったため、もう使えない。

## 3. このコードとROS2の概念(初心者向け)

- **Behavior Tree(BT)**: 「今何をすべきか」を木構造のXMLで表現する仕組み。ノード(木の葉)は
  `SUCCESS`/`FAILURE`/`RUNNING`のいずれかを返し、`Sequence`(全部成功するまで順に実行)や
  `Fallback`(どれか1つ成功するまで順に試す)といった制御ノードでつなぐ。
  Phase 2時点では`bt_xml/bluerov_phase2.xml`(このパッケージ、削除済み)がその実体だったが、
  現在は[behavior_tree/bt_xml/iwakuni2026.xml](../../../behavior_tree/bt_xml/iwakuni2026.xml)
  (§10参照)がその役割を引き継いでいる。
  `bt_executor`(`behavior_tree`パッケージ)が10msごとにこの木を評価し続ける。
- **Lifecycle Node**: `unconfigured → inactive → active`という状態を持つ特殊なノード。
  `zoh_wrench_planner_component`と`emergencyNode`がこれ。BT側の`LifecycleManager`ノードが
  `configure`/`activate`/`deactivate`を`change_state`サービス経由で明示的に呼ぶまで、
  実際の動作(推力計算・緊急浮上)を開始しない。
- **`WrenchPlan`(`planner_msgs/msg/WrenchPlan`)**: 「目標座標(x, y, z, roll, yaw)」を表す
  メッセージ。`targets`が目標(絶対座標)、`master`/`slave`が現在の位置/速度(省略時は
  `wrench_planner`が`odom`から自動で埋める)。**`targets`は自動で埋まらない**ので、
  目標のうち動かしたくない軸(例: 深度だけ変えてx,yはそのまま)は、送信側が現在値を
  明示的にコピーして送る必要がある(Phase 2時点は`GoToDepth`がこれをWrenchPlanのpublishで
  行っていた。現在は`WriteAxisOverrideWaypointCSV`がCSVの1点として同じ考え方を踏襲している。
  §10.6参照)。
- **`zero_order_hold`(ZOH)**: `WrenchPlan`の中継役。`zoh_wrench_plan`で受けた目標を
  `wrench_planner`向けの`goal_current_odom`へそのまま流すが、**送信元(音響班のノード等)からの
  `zoh_wrench_plan`が`timeout_ms`(既定1000ms)以上途絶えると、その瞬間の現在姿勢を目標として
  ホバリング(その場保持)に切り替える**。DVL/odomではなく、あくまで上流の`WrenchPlan`送信の
  途絶を見ている点に注意。
- **`wrench_planner`**: `goal_current_odom`(目標)と`odom`(現在値)からx,y,z,roll,yawの5軸
  独立PIDで力・トルクを計算し、`robot_force`(`geometry_msgs/WrenchStamped`)としてpublishする。
  BlueROVは`mavlink_driver`がこれを受けてPWMへ変換する(`axis.forward/lateral/heave/roll/yaw`)。

## 4. Phase 2で追加したBT葉ノード(履歴。現在生きているのは一部のみ)

| ノード | 役割 | ソース |
|---|---|---|
| `GoToDepth` (**削除済み、§10.6参照**) | 絶対深度(`depth_absolute_m`)または起動時点からの相対オフセット(`depth_offset_m`)へ向かう、自前でWrenchPlanをpublishし続けるノードだった。x,y,roll,yawはodomから読んだ起動時点の現在値を維持。§10.4で一度`z_mode`ポートを追加したが、その後「独自の移動ノードは使わずPDLA経由のCSV方式に統一する」方針が確定し、`WriteAxisOverrideWaypointCSV`(§10.6)に置き換えられて削除された | (削除済み) |
| `GoToBlackboardTarget` (**削除済み、§10.7参照**) | BTブラックボードのキー(`target_x`/`target_y`/`target_z`)が示す絶対座標へ向かう、自前でWrenchPlanをpublishし続けるノードだった。roll/yawは起動時点の現在値を維持。到達判定は`position_tolerance_m`(既定0.2m)/`depth_tolerance_m`(既定0.1m)。`mission_origin_x`/`mission_origin_y`/`field_size_m`によるフィールド境界チェック付き(§6、歴史的経緯)。`GoToDepth`と同じ理由(PDLA経由のCSV方式に統一)で削除された | (削除済み) |
| `CheckPingerFound` (**削除済み**) | `pinger_direction`と`zoh_wrench_plan`を購読し、ハイドロフォンで「発見」したかを判定していた。`CheckPingerPitch`(§10.2)と役割・命名が酷似し混乱していたため削除した | (削除済み) |
| `CheckBuoyDetected` | `buoy_detection`を購読し、confidence/timestampで有効性判定。有効ならodomの姿勢で機体座標系オフセットをNEDへ回転し(tf2で実装。§10.3の`WriteHydrophoneWaypointCSV`もこのパターンを踏襲)、`target_x/y/z`へ出力して`SUCCESS`。`buoy_detection`(`bluerov_control_msgs/msg/BuoyDetection`)の実publisherは`buoy_detector_driver`(`driver/blue_rov/buoy_detector_driver`)。**現在どのXMLからも参照されていない**。実際の画像処理タスク(§10.7)では、`buoy_detector_driver`が未起動でも動く`CheckBuoyPosition`(§10.7、`/buoy/relative_position`直接購読)が採用されたため、このノードの利用計画は無くなった。`buoy_detector_driver`経由の検出パイプラインを使う具体的な計画が無ければ、`CheckBuoyDetected`/`buoy_detector_driver`自体の削除を将来検討してよい | [check_buoy_detected.hpp](include/bluerov_control_bt_nodes/check_buoy_detected.hpp) / [.cpp](src/check_buoy_detected.cpp) |
| `CheckRosBoolParam` (**削除済み**) | ROSパラメータ(`param_name`)の真偽値で即座に`SUCCESS`/`FAILURE`を返す汎用Condition。`bluerov_phase2.xml`専用の2フラグにしか使われておらず、他に使用箇所が無かったため`bluerov_phase2.xml`削除時に一緒に削除した | (削除済み) |
| `CaptureMissionOrigin` (**削除済み**) | `odom`を購読し、DVLが有効ロックしたことを確認して機体位置を`mission_origin_x`/`mission_origin_y`へ確定していた(`GoToBlackboardTarget`の境界チェック用原点、§6)。`bluerov_phase2.xml`専用で他に使用箇所が無かったため、同時に削除した | (削除済み) |

いずれも「未受信・未確定の間は`RUNNING`のまま待機し、誤って`SUCCESS`を返さない」という
安全側の設計を踏襲している。

これらのノードは`behavior_tree`パッケージの`bt_executor.cpp`に追記して登録している
(各ノードにつき`#include`1行+`factory.registerNodeType<...>`1行。既存ノードの登録・動作には
触れていない)。加えてPhase 2では、状態ごとの個別テストのために`bt_executor.cpp`へ
`main_tree_id`という新規ROSパラメータも追加した(空なら従来通りXML自身の
`main_tree_to_execute`属性を使うので、既存挙動に影響はない。このパラメータ自体は
Phase 2削除後も汎用の仕組みとして`bt_executor.cpp`に残っている)。共有パッケージへの
変更なので、取り込む際はメンテナ(Kyubic側)にも一言共有すること。

## 5. `flow.py`の17状態との対応(履歴、Phase 2)

| 旧`flow.py`の状態(群) | BT表現(`bluerov_phase2.xml`) |
|---|---|
| `INIT` | `Delay {init_wait_ms}`(`init_wait_sec`、既定180秒。センサー・ドライバ安定化) |
| (新規追加、旧`flow.py`に対応する状態は無い) | `INIT`直後・`Configure`/`Auto`の前で`Timeout {origin_capture_timeout_ms}`に包まれた`CaptureMissionOrigin`。フィールド境界判定の原点を確定する(§6参照) |
| `DESCEND_TO_7M` | `Auto`内の`GoToDepth`(ポート省略、`default_depth_m`≒`depth_target_m`引数へ) |
| `SEARCH_HYDROPHONE` | `Timeout msec=420000`(7分、`search_timeout_sec`)で`CheckPingerFound`をラップ。時間切れなら`Surface`へ、発見できたらVISUAL/HYDRO分岐へ |
| 発見後の分岐 | `CheckRosBoolParam(debug.image_processing_available)`が真なら`VisualPath`、そうでなければ`HydroPath` |
| `VISUAL_DIVE` | `VisualPath`冒頭の`GoToDepth depth_offset_m="2.5"` |
| `VISUAL_CHECK_MIC`〜`VISUAL_POST_CHECK`ループ | `VisualPath`内、`Timeout msec=45000`(`mic_confirm_timeout_sec`)で`RetryUntilSuccessful`にラップされた`Fallback[ Sequence[CheckRosBoolParam(mic_data_from_above), Surface], Sequence[CheckBuoyDetected, GoToBlackboardTarget, GoToDepth(offset=-0.5), AlwaysFailure] ]`。タイムアウト時は`Emergency`へ(旧`_with_mic_confirm_timeout`と同じ) |
| `HOLD_TARGET` | (素通り。対応するノード無し) |
| `HYDRO_DIVE` | `HydroPath`冒頭の`GoToDepth depth_offset_m="2.5"` |
| `HYDRO_APPROACH`〜`HYDRO_CHECK_MIC`ループ | `HydroPath`内、VISUAL側と同じパターン(`CheckBuoyDetected`は無し。発見時のtarget_x/y/zをそのまま使う) |
| `SURFACE`→`RETURN_HOME`→`DONE` | `Surface` SubTree = `GoToDepth depth_offset_m="-4.0"`。成功=ツリー終端(`RETURN_HOME`/`DONE`は旧実装も無条件成功のスタブだったため、対応するノードは無い) |
| `EMERGENCY` | `Emergency` SubTree(Phase 1から変更なし) |

`AutoSequence`全体は`Timeout {overall_mission_timeout_ms}`(`overall_mission_timeout_sec`、
既定500秒)で覆ってあり、個々のノードが無限に`RUNNING`し続けることが無いようにしている
(§7参照)。500秒は`search_timeout_sec`(420秒、7分)より必ず大きくする必要がある: 内側の
`Timeout[CheckPingerFound]`は`AutoSequence`内の前段処理の分だけ外側より遅れて開始するため、
両者を同じ値にすると探索が内側の期限をほぼ使い切った場合に、内側が自前で`Surface`へ落ちる前に
外側が`AutoSequence`全体を`FAILURE`にしてしまい、本来`Surface`で終わるべき場面が`Emergency`に
なってしまう(既定値500秒 = 420秒 + 前段処理/`VisualPath`・`HydroPath`の
`mic_confirm_timeout_sec`分の余裕80秒)。

## 6. ミッション原点とフィールド境界制限(履歴。`CaptureMissionOrigin`/`GoToBlackboardTarget`は削除済み)

**現状の補足**: このセクションが説明する`CaptureMissionOrigin`(§4)・`GoToBlackboardTarget`
(§10.7)はいずれも削除済み。このフィールド境界チェックという設計自体、現在のコードには
存在しない。将来同様の仕組みが必要になった場合の設計記録として残す。

競技フィールドは10m×10mで、フィールド外の物体を誤検出すると機体が場外へ向かってしまう
リスクがある。これを防ぐため、`GoToBlackboardTarget`(`VISUAL_ATTACK`/`HYDRO_APPROACH`が
向かう先)に境界チェックを追加した。

**設計**: `CaptureMissionOrigin`が`MainTree`の`NormalSequence`冒頭(`Configure`/`Auto`より前、
INIT終了時点)で実行され、そのときの機体位置(odomのx, y)を`mission_origin_x`/
`mission_origin_y`としてブラックボードへ確定する。`GoToBlackboardTarget`は、目標座標が
`[mission_origin_x, mission_origin_x + field_size_m] × [mission_origin_y, mission_origin_y +
field_size_m]`(既定`field_size_m=10.0`)の範囲外なら、`publish`せず`FAILURE`を返す
(誤検出扱いとして、既存の`RetryUntilSuccessful`ループへ自然に戻る。新しい分岐ロジックは
不要)。範囲は原点から**+x方向・+y方向へ非対称**に伸びる矩形であり、原点を中心とした
±5mの対称範囲ではない。

**⚠️ 運用手順(コードでは担保していない、必ず守ること)**: +x/+yがフィールド奥方向を
向くことは、**機体を投入地点でフィールド奥を向けて起動する**という運用手順のみで担保して
いる。コード側でのヨー角補正は行っていない。起動時の向きを間違えると、境界チェックが
実際のフィールドと無関係な方向を基準にしてしまい、正しく機能しない。

**⚠️ 未検証リスク(実機テスト前に必ずプールで確認すること)**: `CaptureMissionOrigin`は
競技規則上ひざ程度の浅い投入水深、かつ機体が潜航を開始する前(`DescendTo7m`より前)に
実行される設計になっている。

- DVLが、この浅い水深でビームロックできるかは**未検証**。ロックできない場合、
  `origin_capture_timeout_sec`(既定60秒)が競技本番でも毎回タイムアウトし、
  `Emergency`へ落ちてしまうリスクがある
- 実機テスト前に必ず、投入直後のDVLロック状況(`status.dvl.id`が`NORMAL`になるか、
  なるまでに何秒かかるか)を浅いプールで計測・確認すること(§8にプールテストの
  確認項目として記載)
- 計測の結果、ロックできない、または時間がかかりすぎる場合は、`CaptureMissionOrigin`の
  呼び出し位置を`DescendTo7m`後(`Auto`内、より深い水深に到達した後)へ移す設計変更が
  必要になる可能性がある。**現時点ではこの設計変更は行っておらず**、次の検証結果待ち

`CaptureMissionOrigin`自体は個別のタイムアウトを持たない(`GoToDepth`等と同じ設計方針)ため、
`MainTree`の`NormalSequence`内で明示的に`Timeout {origin_capture_timeout_ms}`
(`origin_capture_timeout_sec`、既定60秒)に包んである。この区間は`overall_mission_timeout_ms`
(`Auto`の内側のみを覆う)の対象外のため、別のタイムアウトが必要だった。タイムアウトすれば
`NormalSequence`全体が`FAILURE`となり、既存の`Fallback[NormalSequence, Emergency]`構造が
そのまま`Emergency`へ落とす。

## 7. 既知の制約・TODO(現行、iwakuni2026.xml時点)

Phase 2固有だった項目(`CaptureMissionOrigin`の浅水深DVLロック検証、`AutoSequence`の
タイムアウト設計など)は、該当ノード・XMLの削除により対象が無くなったため以下から除いた。
現在も有効な制約のみ残す:

- **Part A/B/Cのプール再検証が未完了**。DVL/odomゼロ値伝播バグの修正(§9)自体は現行コードに
  残っているが、iwakuni2026.xmlでの実機再検証はまだ行われていない
- `BatteryCheck`/`CheckSensorsStatus`は未組み込みのまま。Kyubicの閾値(4S/6S電池パック前提)が
  BlueROVにそのまま使えるか未検証のため
- PIDゲイン([config/p_pid_controller_gain_bluerov.yaml](config/p_pid_controller_gain_bluerov.yaml))は
  実機未チューニングの保守的な初期値のまま。特にroll軸は旧`bluerov_control`が一度も
  制御したことがない軸なので出力レンジを大きく絞ってある
- `sbl_controller_node`(音響班、`pinger_direction`を発行する側)は引き続き外部/未実装。
  `CheckPingerPitch`/`WriteHydrophoneWaypointCSV`は`pinger_direction`の入力トピックが
  来ない限り安全側(`FAILURE`/`RUNNING`)で待機し続ける(§10.2/10.3)
- `CheckBuoyDetected`が購読する`buoy_detection`は、中継アダプタ`buoy_detector_driver`
  (`driver/blue_rov/buoy_detector_driver`)により実publisherが存在するようになった。ただし
  上流の`perception/buoy_detector`(YOLO検出)自体はYOLOモデル・水中カメラ校正が未整備のため
  まだ実行できない(詳細は`buoy_detector_driver/README.md`§4)。ただし`CheckBuoyDetected`
  自体は現在どのXMLからも参照されていない(§4参照)
- 旧`bluerov_control`(`flow.py`/`control.py`)は、BT構成(現iwakuni2026.xml)が実機で
  一通り検証できてから正式に廃止する予定(それまでは両方の起動経路が残る)
- **odom鮮度の監視が無い**: `ZeroOrderHold`は`zoh_wrench_plan`(目標)側の途絶しか監視しておらず、
  `odom`(現在値)側の鮮度は誰も見ていない。§9のPart A修正でDVL/depth/IMUがERROR時に直前値を
  保持するようになった結果、これらのセンサが長時間沈黙し続けると、`odom`が同じ値のまま
  「凍りついた」状態がそれと分からず流れ続けるという新たなリスクがある。対策案
  (`localization_component`が各センサのERROR継続時間を計測し閾値超過でさらなるフラグを立てる、
  `wrench_planner`側で`odom.header.stamp`の鮮度をチェックする等)は未実装、次回以降の検討事項
- `transit_shallow.csv`/`descend_deep.csv`(iwakuni2026.xmlが参照する静的CSV)がまだ
  `path_planner/assets/iwakuni/`に用意されていない(§10.3)。実機の現地座標が必要

## 8. プールテストの進め方(削除済み。§10.5に現行版あり)

Phase 2の個別SubTreeテスト(`main_tree:=MainTree_SearchOnly`等)は、その対象だった
`bluerov_phase2.xml`自体が削除されたため、このセクションは丸ごと削除した。汎用的に
使えるダミーデータ注入コマンド・緊急浮上の単体テスト手順は§10.5へ移した。

## 9. プール実験で発見された不具合の修正(2026年)

初回プール実験で「DVLが壁に近づくと`/localization/odom`にゼロ値が出て、PID制御が振動する」
事象が発生した。原因は、DVL/depth/IMUのいずれかが計測エラー(`status.<sensor>.id == ERROR`)に
なった際、**エラーフラグは正しく伝播していたのに、値そのものはゼロのまま無条件で
下流(最終的に`wrench_planner`が使う`/localization/odom`)まで伝播していた**こと
(`dvl75_driver` → `dvl_odometry_component` → `localization_component`の3層、
DVL/depth/IMUの3センサ、計5箇所で同じパターンが見つかった)。

修正方針: **「ERRORの入力は対応するフィールドを更新せず、直前の有効値を保持する。
ステータスフラグ自体はそのまま伝播させる」**に統一。フラグで異常を検知しつつ、
値は「最後に分かっている最良の推定値」であり続けるようにした。

- [dvl75_driver.cpp](../../../driver/blue_rov/dvl75_driver/src/dvl75_driver.cpp): confidence/
  ビームロック不足時とパース失敗時の両方で、直前の有効な速度・高度を保持してpublishするように
  変更。パース失敗時は診断用の`dvl75`トピックにも(以前は何も送られていなかったが)最小限の
  エラーメッセージを送るようにした。
- [dvl_odometry_component.cpp](../../../localization/localization/src/dvl_odometry_component.cpp):
  ERROR時にpublishするodomの位置・速度が(積算値`pos_x`/`pos_y`は保持されているのに)ゼロの
  ままpublishされていたのを、直前値をセットするよう修正。
- [localization_component.cpp](../../../localization/localization/src/localization_component.cpp):
  `depth_callback`/`imu_callback`/`dvl_callback`の3つとも、対応するセンサが`ERROR`の間は
  フィールド更新をスキップし、直前の融合済み値を保持するよう修正(3つとも同じ欠陥だった)。

## 10. iwakuni2026コース(現行、BlueROVミッション制御の唯一の構成)

`behavior_tree`パッケージの[bt_xml/iwakuni2026.xml](../../../behavior_tree/bt_xml/iwakuni2026.xml)
(`BlueROV2Auto`ツリー)が、BlueROVミッション制御の現行構成。旧`bluerov_phase2.xml`
(§1〜9、GoToDepth/GoToBlackboardTarget/CheckBuoyDetected等による状態遷移方式)は
iwakuni2026.xmlへ統合され**廃止・削除された**(§10.1)。iwakuni2026.xmlは`WaypointAction`
(PDLA、`path_planner`のCSV経路追従)を主体に、ハイドロフォン方向へ0.5mずつ**その場でCSVを
書き換えながらステップ移動**する方式を採る。`sbl_controller_node`(音響班、外部・未実装)
には依存しない。

### 10.1 Phase 1/Phase 2の廃止

`iwakuni2026.xml`との役割重複が明確になったため、以下を削除した(履歴は§1〜9に残す):

- `bt_xml/bluerov_phase1.xml` / `bt_xml/bluerov_phase2.xml`(個別テスト用の
  `MainTree_VisualPath`等のSubTreeも、これらのファイルの一部だったため同時に削除)
- `CheckPingerFound`(§4。`CheckPingerPitch`と役割・命名が酷似し混乱していたため)
- `CaptureMissionOrigin`・`LoadMissionParams`・`CheckRosBoolParam`(§4。いずれも
  `bluerov_phase2.xml`専用で、削除後は他に使用箇所が無いため)
- `config/bluerov_mission.param.yaml`(`LoadMissionParams`専用の定数ファイル)

**残したもの(当時)**: `GoToDepth`(当時iwakuni2026.xmlで現役使用。後に`WriteAxisOverrideWaypointCSV`
へ置き換えられて削除された。§10.6参照)、`GoToBlackboardTarget`(後に同じ理由で削除された。
§10.7参照)・`CheckBuoyDetected`(§4参照。今後の画像処理タスクでは代わりに`CheckBuoyPosition`
が採用されたため、現在も未使用のまま)。

`bluerov_bt.launch.py`(§10.4)は、`bt_xml_file`のデフォルトを`bluerov_phase2.xml`から
`iwakuni2026.xml`(`behavior_tree`パッケージ側)へ更新し、`mission_config`
(削除した`bluerov_mission.param.yaml`)の読み込みも外した。個別SubTreeテスト用の
`main_tree`launch引数自体は汎用の仕組みとして残してあるが、テスト対象だった
`MainTree_VisualPath`等のSubTreeはもう存在しない。

### 10.2 `CheckPingerPitch`(Condition)

`pinger_direction.pitch`が`pitch_threshold_deg`(既定50.0)以下なら`SUCCESS`
(もう十分近い)、そうでなければ`FAILURE`。`pinger_direction`を一度も受信していない
間も`FAILURE`(安全側、まだ遠いとみなす)。

`CheckPingerFound`等と違い`BT::ConditionNode`として実装した(`RUNNING`を返さない)。
理由: `iwakuni2026.xml`では`Fallback`の1番目の子として使われており(§10.4のXML参照)、
もし`RUNNING`を返すとFallbackが2番目の子(ステップ移動)を一切試せなくなってしまう。
毎tick即座にSUCCESS/FAILUREを返す必要がある。

**【未検証】**: `pitch <= threshold`で「近い」と判定する向き自体は、依頼者が
iwakuni2026.xml側で決めた仕様をそのまま実装したもの。`sbl_estimation_node`
(音響班)側の`pitch`正方向の定義と組み合わせて、実機で必ず検証すること。

### 10.3 `WriteHydrophoneWaypointCSV`(Action)

現在のodomと`pinger_direction`(機体座標系の相対yaw/pitch、度)から、
`step_distance_m`(既定0.5m)先の1点を計算し、`path_planner`が読めるCSV形式で
`csv_file_path`へ書き出す。odom/pinger_directionのどちらかを一度も受信していない
間は`RUNNING`で待機し(GoToDepth等と同じ安全側の設計)、書き込みが成功したら
1回で`SUCCESS`を返す(移動の完了は待たない。移動自体は後続の`WaypointAction`が担う)。

**変換の考え方**: `pinger_direction`のyaw/pitch(機体座標系の相対角度)を、まず
機体座標系のオフセットベクトル(球面座標→直交座標)に変換し、`CheckBuoyDetected`
(§4参照)と同じ`tf2::Quaternion::setRPY` + `tf2::quatRotate`のパターンで、
現在のodomの姿勢(度→ラジアン変換したroll/pitch/yaw)を使ってNED絶対座標へ回転する。
x/yはこうして得た絶対座標、rollは常に現在値を維持(GoToDepth/GoToBlackboardTarget
と同じ方針)、z_modeは常に`Z_MODE_DEPTH`(水平方向の探索中は深度を維持する想定)。

**yawの扱い(2パターン)**: `step_distance_m > 0`(通常のステップ移動、§10.2の
`PingerStepAndRetry`)では、rollと同様に現在のyawを維持する(向きは変えず位置だけ
ピンガー方向へ進める)。一方`step_distance_m == 0.0`の場合のみ、`target_yaw`を
`current_yaw + pinger_direction.yaw`(ピンガー方向)へ切り替える。これは§10.5の
`CheckBuoyInFrame`と組み合わせた「その場で向き直る」用途(`TurnTowardPinger`)のためで、
`step_distance_m == 0.0`なら位置オフセットは自動的にゼロになる(x/y/zは現在地のまま)
ため、実質「その場で回頭するだけ」の1点CSVになる。歩幅0はゼロ除算等の危険な演算を
一切経由しないため安全(§10.5参照)。

**【未検証・符号反転ポイント】**: `pinger_direction.pitch`は「上向きが正」という
仮定で実装している(依頼者確認済みの仮定。ただし`sbl_estimation_node`側の実装依存
であり、コード側だけでは断定できない)。実機で符号が逆だと判明した場合、
[write_hydrophone_waypoint_csv.cpp](src/write_hydrophone_waypoint_csv.cpp)内の
`body_offset_z`計算(1行)の符号を反転させれば直るように実装してある。

**CSVフォーマット**: `path_planner`(`PathCsvLoader`)は19列固定・ヘッダー完全一致の
独自CSV形式を要求する([path_csv_loader.hpp](../../planning/path_planner/include/path_planner/path_csv_loader.hpp)参照)。
1点だけの経路でも、ヘッダー行+7個の固定パラメータ行(`checkpoint_end_row=1`,
`catmull_rom=0`, `catmull_end_row=0`, ...)+1個のcheckpointデータ行が必要。
書き込みは一時ファイル+`std::filesystem::rename`による原子的な置き換えとし、
`WaypointAction`(PDLA)が書き込み途中の不完全なファイルを読んでしまうレース
コンディションを防いでいる。

**パス解決**: `csv_file_path`が`/`始まりでなければ、`pdla_planner.cpp`と同じ規則
(`ament_index_cpp::get_package_share_directory("path_planner")`を基準に結合)で
絶対パスへ解決する。`WaypointAction`が最終的に同じファイルを読むため、解決規則を
合わせる必要がある。書き込み先ディレクトリが無ければ`std::filesystem::create_directories`
で作成するため、`assets/iwakuni/`ディレクトリ自体が事前に無くても動的書き込み分
(`hydrophone_step.csv`)は問題ない。

**運用上の注意(CMakeの再configure)**: 一方、`transit_shallow.csv`/`descend_deep.csv`
のような**静的**なCSV(依頼者が別途`path_planner/assets/iwakuni/`配下に用意する想定、
本タスクの成果物には含まない)は、`path_planner/CMakeLists.txt`の
`install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})`という素朴な指定のため、
ファイルを追加しただけでは`colcon build`を再実行してもインストール(symlink)に
反映されないことがある。新しい静的CSVを追加した後は、`build`/`install`ディレクトリ
ごと消してからのクリーンビルドで運用すること(`CMakeLists.txt`自体の変更は不要)。

### 10.4 `GoToDepth`への`z_mode`ポート追加(履歴。`GoToDepth`は削除済み。§10.6参照)

**この節は過去の記録**。この時点では既存の`GoToDepth`を流用する方針だったが、その後
「独自にWrenchPlanを出し続ける移動ノードは使わず、PDLA経由のCSV方式に統一する」方針が
確定し、`GoToDepth`自体が`WriteAxisOverrideWaypointCSV`(§10.6)へ置き換えられて削除された。
`z_mode`ポート(文字列`"depth"`/`"altitude"`)という考え方自体は`WriteAxisOverrideWaypointCSV`
にも引き継がれている。

`iwakuni2026.xml`のハイドロフォン追跡ループの直後、「今のx/y/roll/yawは維持したまま、
水底からの高度を2.5mへ変える」という最終潜航に、当時は既存の`GoToDepth`を流用した:

```xml
<GoToDepth name="PingerFinalDescend" depth_absolute_m="2.5" z_mode="altitude" tolerance_m="0.2" />
```

`GoToDepth`は元々`z_mode`を持たず、常に`pose.position.z_depth`(水深基準)・
`WrenchPlan::Z_MODE_DEPTH`固定で動いていた。今回`z_mode`ポート(文字列
`"depth"`(既定)/`"altitude"`)を追加し、`"altitude"`指定時は`pose.position.z_altitude`
(水底からの高度)を現在値として読み、publishする`WrenchPlan.z_mode`も
`Z_MODE_ALTITUDE`に切り替えるようにした。`z_mode`未指定時は従来通り
depth基準で動作するため、`z_mode`ポートを指定しない既存呼び出しへの後方互換は保たれている。

### 10.5 `CheckBuoyInFrame`(Condition)

`Final Descend Successful`(最終潜航完了)直後、`BuoyInFrameOrTurn`ループで使う。
`CheckPingerPitch`(§10.2)と全く同じ実装パターンの`BT::ConditionNode`
(`RUNNING`を返さない。親の`Fallback`が2番目の子(向き直り)を毎tick試せるようにするため)。

`buoy_interfaces/msg/BuoyRelativePosition`を**直接**購読する点に注意。前回作った
`buoy_detector_driver`(`driver/blue_rov/buoy_detector_driver`、`BuoyRelativePosition`を
`bluerov_control_msgs/msg/BuoyDetection`へ変換する中継アダプタ。`CheckBuoyDetected`が
購読する経路)とは**別経路**で、`CheckBuoyInFrame`は画像処理班の`perception/buoy_detector`
(`buoy_detector_node.py`)が発行する生データをそのまま見る。トピック名
`/buoy/relative_position`は`buoy_detector_node.py`の実際のpublish先
(`buoy_detector.launch.py`のデフォルト値)と一致することを確認済み。

`detected == true`かつ`confidence >= confidence_threshold`なら`SUCCESS`、それ以外
(未受信の場合を含む)は`FAILURE`(`CheckPingerPitch`と同じく安全側)。

**`confidence_threshold`(既定0.5)は暫定値**: 実機・実データでの調整が前提。
YOLO検出のconfidenceスケールは`perception/buoy_detector`側のモデル・閾値設定に依存する
ため、実際の検出結果を見ながら調整すること。

**`TurnTowardPinger`(`step_distance_m=0.0`)の安全性検証結果**: `WriteHydrophoneWaypointCSV`
の歩幅は係数として直接掛けているだけで正規化・除算を一切行わないため、`step_distance_m=0.0`
でもゼロ除算やNaNは発生しない(§10.3参照)。また書き込むCSVは常にちょうど1点の
checkpointのみのため、`pdla_planner.cpp`側の複数点経路専用のベクトル正規化コード
([pdla_planner.cpp:320-324](../../planning/projection_dynamic_look_ahead_planner/src/pdla_planner.cpp#L320-L324)、
中間点でのみ実行されるコード)も実行されない。ただし当初の実装は`target_yaw`を常に
現在値で維持していたため、歩幅0だと「x/y/z/roll/yawが全て現在と同じ」＝完全な
無意味な1点になり、**回頭が一切起きない**という設計上の不具合があった。これを受けて
§10.3の通り、`step_distance_m == 0.0`の場合だけ`target_yaw`をピンガー方向へ切り替える
よう修正した。

### 10.6 `WriteAxisOverrideWaypointCSV`(Action)と`GoToDepth`/`GoToX`の廃止

iwakuni2026.xmlの設計方針が「独自にWrenchPlanを出し続ける移動ノード(`GoToDepth`のような
実装)は使わず、現在odom基準の1点CSVを書いて既存の`WaypointAction`(PDLA、Kobe実大会での
実績がある経路)に処理を任せる」方式へ統一されたことを受け、`GoToDepth`を汎用の
「軸ごとの部分上書き」ノード`WriteAxisOverrideWaypointCSV`へ置き換えた。

```xml
<!-- 例1: 深度(高度)だけ上書き -->
<WriteAxisOverrideWaypointCSV
  odom_topic_name="odom"
  override_z="2.5"
  z_mode="altitude"
  csv_file_path="assets/iwakuni/pinger_final_descend.csv" />
<WaypointAction action_name="planner/pdla_planner/pdla_plan"
  csv_file_path="assets/iwakuni/pinger_final_descend.csv" />
```

`override_x`/`override_y`/`override_z`/`override_yaw`のうち**指定されたポートだけ**その値を
使い、指定されなかった軸は起動時点のodomの値をそのまま使う(部分上書き)。roll軸はこの
ノードの上書き対象ではなく、常に現在値を使う。ポート未指定の判定は`GoToDepth`の
`depth_absolute_m`/`depth_offset_m`と全く同じ方法(`BT::InputPort`にデフォルト値を
与えず、`getInput()`の戻り値をbool変換して判定)を踏襲している。

**z軸の基準**: `z_mode`(`"depth"`(既定)/`"altitude"`)で選んだ基準に応じて、`override_z`
未指定時のフォールバック値も`z_altitude`/`z_depth`のどちらかに切り替わる(`GoToDepth`の
`z_mode`ポートと同じ考え方をそのまま踏襲)。CSVの`z_mode`列は1行につき1つしか無いため、
「オーバーライドされた値」と「現在値からのフォールバック」で基準がずれないようにしている。
仕様書には「z(depth基準)」とだけ書かれていたが、`z_mode="altitude"`かつ`override_z`未指定
という(今回の2例には現れない)組み合わせでも矛盾なく動くよう、`GoToDepth`の既存パターンに
合わせてこのように実装した。

odomを一度も受信していない間はRUNNINGのまま待機する(`GoToDepth`/`CaptureMissionOrigin`と
同じ安全側の設計)。書き込みが成功したら1回でSUCCESSを返す(実際の移動・完了判定は後続の
`WaypointAction`が担う)。

**CSV書き込みロジックの共通化**: `WriteHydrophoneWaypointCSV`(§10.3)が個別に持っていた
「19列固定フォーマットへの書き込み」「`pdla_planner.cpp`と同じパス解決規則」「atomic
write(一時ファイル+rename)」を、`waypoint_csv_writer::ResolvePath`/
`::WriteSingleCheckpoint`という共通ヘルパー([waypoint_csv_writer.hpp](include/bluerov_control_bt_nodes/waypoint_csv_writer.hpp)/[.cpp](src/waypoint_csv_writer.cpp))
へ切り出した。BTノードではない(`bt_executor.cpp`への登録は無い)単なる関数群で、
`WriteHydrophoneWaypointCSV`・`WriteAxisOverrideWaypointCSV`の両方がこれを呼ぶ形に
リファクタリングした(`WriteHydrophoneWaypointCSV`側の座標変換ロジック自体は変更なし)。
今後3つ目以降の「1点CSV書き込み」ノードが必要になっても、この共通ヘルパーを再利用できる。

**`GoToDepth`の削除**: `.hpp`/`.cpp`を削除し、`bt_executor.cpp`の登録も削除した。
`bluerov_bt.launch.py`の`depth_target_m`launch引数・`default_depth_m`パラメータ上書き
(`GoToDepth`のポート省略時フォールバック専用だった)も、参照する相手が無くなったため
あわせて削除した。

**`GoToX`について**: 探索の結果、`GoToX`はC++の実装・`bt_executor.cpp`への登録のいずれも
**存在しなかった**(`iwakuni2026.xml`の`TreeNodesModel`にモデル定義があっただけ)。そのため
コード側での削除作業は発生していない(モデル定義の削除だけで完了している)。

### 10.7 画像処理タスク: 中心合わせ→突撃→解放判定(`FindPingerAction`成功後)

`iwakuni2026.xml`の`FindPingerAction`成功直後に、ブイへの接近〜接触〜解放判定の4フェーズを
追加した:

```
FindPingerAction成功
  ↓
Timeout{180000ms}[ RetryUntilSuccessful[ Fallback ]]        … ①中心合わせ/②突撃
  Fallback "CenteringOrFinalApproach":
    Sequence "FinalApproach"（ブイに十分近い時だけ成立、1回のみ）:
      CheckBuoyPosition → WriteAxisOverrideWaypointCSV → WaypointAction
    Sequence "CenteringStep"（それ以外は毎回こちら、繰り返し）:
      WriteVisionWaypointCSV → WaypointAction → AlwaysFailure
  ↓
Delay{10000ms}                                                … ③接触後の停止(ZOHが自動ホールド)
  ↓
Timeout{30000ms}[ RetryUntilSuccessful[ Sequence ]]          … ④解放判定
  Sequence "ReleaseConfirmed":
    Inverter[CheckPingerPitch(pitch_threshold_deg=70.0)]
    Inverter[CheckBuoyInFrame]
```

#### `GoToBlackboardTarget`の削除

`GoToDepth`と同じ理由(PDLAを経由しない独自のWrenchPlan送出ノードは使わない方針)で、
`GoToBlackboardTarget`(`.hpp`/`.cpp`、`bt_executor.cpp`の登録)を削除した。`iwakuni2026.xml`
の`TreeNodesModel`にはそもそもモデル定義が無かった(本体でも未使用)ため、XML側の削除作業は
発生していない。

**参考(現状の課題として記録)**: `GoToBlackboardTarget`は元々`CheckBuoyDetected`
(`bluerov_control_msgs/msg/BuoyDetection`を`buoy_detector_driver`経由で購読するノード、§4)
の出力先として設計されていたが、今回の`FinalApproach`は`CheckBuoyDetected`を使わず新規の
`CheckBuoyPosition`(下記)にした(理由は次項参照)。結果、`CheckBuoyDetected`自体も現時点で
呼び出し元が無く休眠したままになっている。`buoy_detector_driver`経由の検出パイプラインを
今後使う具体的な計画が無ければ、`CheckBuoyDetected`/`buoy_detector_driver`自体の削除を
将来検討してよい(今回のタスクスコープ外のため削除はしていない)。

#### `CheckBuoyPosition`(Action)の新設と、`CheckBuoyDetected`を使わなかった理由

当初案では`FinalApproach`に既存の`CheckBuoyDetected`を流用する想定だった。探索の結果、
以下2点の問題が見つかったため、`CheckBuoyInFrame`/`WriteVisionWaypointCSV`と同じく
`buoy_interfaces/msg/BuoyRelativePosition`を直接購読する新規ノード`CheckBuoyPosition`を
作った(依頼者確認済み):

1. `CheckBuoyDetected`のトピックポート名は`buoy_detection_topic`(既定`"buoy_detection"`、
   相対名)であり、`buoy_relative_position_topic`という名前のポートは存在しない。当初案の
   XML草案にあった`buoy_relative_position_topic="/buoy/relative_position"`という指定は、
   このノードには効かない属性だった
2. `CheckBuoyDetected`は`bluerov_control_msgs/msg/BuoyDetection`(`buoy_detector_driver`が
   `BuoyRelativePosition`から変換して発行する)を購読する設計で、`bluerov_bt.launch.py`は
   その相対トピック`buoy_detection`を`/perception/buoy_detection`へremapする準備はあるが、
   **`buoy_detector_driver`自体を起動するlaunchファイルがどこにも無い**。そのため実際に
   動かすと、`FinalApproach`は`RUNNING`のまま永遠に進まなくなる状態だった

`CheckBuoyPosition`は`CheckBuoyDetected`と全く同じ変換ロジック(tf2による機体座標系→NED回転、
`target_x`/`target_y`/`target_z`出力)を、`CheckBuoyInFrame`/`WriteVisionWaypointCSV`と同じ
`/buoy/relative_position`直接購読の経路で行う。`detected`/`position_valid`/`confidence`/
`stale_timeout_sec`の全てをゲート条件とする(`position_valid=false`だとx/y/zがNaNのため
必須)。

#### `WriteVisionWaypointCSV`(Action、`CenteringStep`用)

`WriteHydrophoneWaypointCSV`と構造は同じ(odom/検出を待つ→計算→`waypoint_csv_writer`で
atomic write)だが、入力メッセージ型と、水平/垂直で別の情報源から補正量を作る点が異なる。
詳しい計算手順は[write_vision_waypoint_csv.hpp](include/bluerov_control_bt_nodes/write_vision_waypoint_csv.hpp)
のコメントを参照。要点:

- **yaw補正**: `yaw_valid`なら`relative_buoy_yaw_rad`をそのまま使う。そうでなければ
  `normalized_horizontal_offset × (horizontal_fov_rad / 2)`で近似する。`deadband_rad`
  未満の補正は0にする(ジッター防止)。
- **【設計変更・実機要検証】垂直補正(z_step)**: 当初案は`normalized_vertical_offset`
  (無次元、画像中心からの正規化縦偏差)を使う想定だったが、`buoy_interfaces/msg/
  BuoyRelativePosition`にはこのフィールドが**存在しない**(水平方向の
  `normalized_horizontal_offset`に対応する垂直版が無い)。依頼者と協議の上、
  `relative_buoy_z_m`(機体座標系の相対深度[m]、`position_valid=true`の時のみ有効)で
  代用することにした。`vertical_gain_m`(既定0.2)は元々「無次元の正規化偏差に掛ける
  メートル単位の係数」だったが、`relative_buoy_z_m`は既にメートル単位のため、
  **無次元の比例ゲイン**(1tickでこの割合だけ縦方向の推定誤差に近づく、という意味)として
  再解釈して使っている。符号も`relative_buoy_z_m`(機体座標系で正=下方向)を反転せずそのまま
  使う形に作り直した(元の式は正規化偏差の符号を反転させていたが、これは画像座標系からNED
  座標系への変換のためであり、`relative_buoy_z_m`は最初からNED慣習と一致する前提のため不要と
  判断した)。**この解釈・符号は実機で必ず検証すること**。`position_valid=false`の間は
  z_step=0とし、水平方向の中心合わせのみ進める。
- x/y方向は、新しいyaw(現在yaw+補正)の向きへ`step_distance_m`だけ前進する形で計算する
  (`WriteHydrophoneWaypointCSV`のような球面座標→body→NED回転ではなく、単純な2D平面上の
  cos/sinで済む。yawの単位変換に注意: odomのyawは度、`BuoyRelativePosition`関連の角度は
  ラジアンのため、混在させず内部でラジアンに統一してからCSVの度単位へ変換して書き出す)。

#### 閾値・パラメータ一覧(いずれも暫定値。実験調整前提)

| パラメータ | 暫定値 | 備考 |
|---|---|---|
| `step_distance_m`(`WriteVisionWaypointCSV`) | 0.3 | ハイドロフォン用(0.5)より小さめ |
| `vertical_gain_m` | 0.2 | 無次元の比例ゲイン(上記参照。当初「メートル」想定から再解釈) |
| `deadband_rad` | 0.035(約2°) | ジッター防止の不感帯 |
| `horizontal_fov_rad` | 未定 | 画像処理班のカメラ校正値待ち。届くまでポート必須のまま仮値で運用 |
| `confidence_threshold` | 0.5 | 既存の`CheckBuoyInFrame`と揃える |
| `vision_approach_timeout_ms`(①②ループ全体) | 180000(180秒) | 依頼者提案の暫定案をそのまま採用 |
| `release_check_timeout_ms`(④解放判定) | 30000(30秒) | 依頼者からの明示指定は無く、①②より短い暫定値として置いた |

#### ④解放判定の閾値の符号について(重要・経緯を記録)

当初「pitch 20°でSUCCESS」という案があったが、**依頼者と協議の結果、(b)`<Inverter>`で
`CheckPingerPitch`を包む方式に確定した**。`CheckPingerPitch`自体は変更せず(`pitch ≤ 閾値`で
`SUCCESS`のまま)、`<Inverter>`で包むことで「`pitch ≥ 閾値`でSUCCESS(ピンガーがほぼ真上=
解放してよい)」という意味になる。この方式変更に伴い、**閾値の意味が「近い」から「離れた
(≒真上)」へ反転したため、20°ではなく70°前後の大きい値に置き換えた**。70°もまた実測データ
の無い暫定値であり、実験で調整すること。

**参考(未検証点)**: `CheckPingerPitch`/`CheckBuoyInFrame`はいずれも、対応するトピックを
一度も受信していない間は安全側で`FAILURE`を返す設計(§10.2/10.5)。`<Inverter>`で包むと、
この「未受信時のFAILURE」が「未受信時のSUCCESS」に反転する。今回の④解放判定は
`FindPingerAction`成功後、既に`CenteringOrFinalApproach`ループで両トピックを長時間
購読し続けた後に到達するため、実運用では「未受信」状態にはならない想定だが、単体で
④のSubTreeだけを切り出してテストする場合はこの反転に注意すること。

### 10.8 動作確認のヒント(暫定)

`sbl_controller_node`(音響班)、および実機のYOLO検出(`perception/buoy_detector`、モデル・
カメラ校正が未整備)がまだ無いので、`CheckPingerPitch`/`WriteHydrophoneWaypointCSV`/
`CheckBuoyInFrame`を動かすには別ターミナルでダミーデータを注入する(機体を拘束した状態で):

```bash
# ハイドロフォン方向のダミー値(CheckPingerPitch/WriteHydrophoneWaypointCSVが購読)
ros2 topic pub /pinger_direction planner_msgs/msg/PingerDirection \
  "{yaw: 0.0, pitch: 35.0, score: 0.01}" --rate 5

# ブイ検出のダミー値(CheckBuoyInFrameが直接購読。buoy_detector_driverは経由しない)
ros2 topic pub /buoy/relative_position buoy_interfaces/msg/BuoyRelativePosition \
  "{detected: true, yaw_valid: true, position_valid: true, confidence: 0.9}" --rate 5
```

緊急浮上だけを単体で再現する場合、`bluerov_bt.launch.py`起動後、別ターミナルで
`emergencyNode`のLifecycle状態を直接操作する。BT側の`LifecycleManager`が内部でやっているのと
同じ操作(`change_state`サービス呼び出し)をCLIから直接呼ぶだけ(新規コード不要)。
`ros2 lifecycle set`はその定型ラッパーで、生のサービス呼び出し
(`ros2 service call .../change_state lifecycle_msgs/srv/ChangeState "{transition: {id: ...}}"`)
より短く打てる:

```bash
ros2 lifecycle set /emergencyNode configure
ros2 lifecycle set /emergencyNode activate
```
