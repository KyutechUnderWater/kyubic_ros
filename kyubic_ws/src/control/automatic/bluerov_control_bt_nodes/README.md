# bluerov_control_bt_nodes

BlueROVをKyubicと同じ **Behavior Tree(BT) + `wrench_planner`(P-PID) + `zero_order_hold`(ZOH) +
`emergency`** 構成で動かすための、BlueROV固有の追加分。

**Phase 2(このドキュメントの現状)**: 旧`bluerov_control/flow.py`の17状態すべてをBT XML
(`bt_xml/bluerov_phase2.xml`)へ移行済み。Phase 1(`bluerov_phase1.xml`、固定深度潜航のみ)は
参考として残っているが、`bluerov_bt.launch.py`はPhase 2のXMLを使う。

**重要な注意(未検証リスクその1)**: このPhase 2着手時点で、Phase 1のDVL/odomゼロ値伝播バグ修正
(§9のPart A/B/C)は**プールでまだ再検証されていない**。依頼者はこれを把握した上でPhase 2の実装を
進めることを明示的に選択した。実機テストの前に必ずPart A/B/Cの再検証を先に行うこと。

**重要な注意(未検証リスクその2)**: `CaptureMissionOrigin`(投入直後、浅い水深でDVLロックを
待ってフィールド境界判定の原点を確定するノード)は、**その水深でDVLが実際にビームロックできるか
プールで未検証**。ロックできない、または時間がかかりすぎる場合、`origin_capture_timeout_sec`
(既定60秒)が競技本番でも毎回タイムアウトし`Emergency`へ落ちるリスクがある。詳細と実機テスト前に
必須の確認事項は§6参照。

対象読者: ROS2に触れ始めたばかりの開発者。

## 1. なぜこの構成にしたか

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

## 2. 起動方法

```bash
# 1. BlueROVのハードウェアドライバ
ros2 launch driver_launcher blue_rov_driver.launch.py

# 2. センサ融合(位置・姿勢推定)
ros2 launch localization localization_components.launch.py

# 3a. 【Phase 2: BT構成、通しでMainTreeを実行】このパッケージ
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py depth_target_m:=1.0

# 3a'. 【個別テスト】特定のSubTreeだけを単体起動する(§8参照)
# ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=MainTree_VisualPath

# 3b. 【旧構成、切り戻し用】従来のFSM+PID(3a/3a'とは同時に起動しないこと。両方がrobot_forceを
#     送ろうとして競合する)
# ros2 launch bluerov_control bluerov_control.launch.py
```

## 3. このコードとROS2の概念(初心者向け)

- **Behavior Tree(BT)**: 「今何をすべきか」を木構造のXMLで表現する仕組み。ノード(木の葉)は
  `SUCCESS`/`FAILURE`/`RUNNING`のいずれかを返し、`Sequence`(全部成功するまで順に実行)や
  `Fallback`(どれか1つ成功するまで順に試す)といった制御ノードでつなぐ。
  [bt_xml/bluerov_phase2.xml](bt_xml/bluerov_phase2.xml)がその実体
  ([bt_xml/bluerov_phase1.xml](bt_xml/bluerov_phase1.xml)はPhase 1の最小構成版で参考用に残置)。
  `bt_executor`(`behavior_tree`パッケージ)が10msごとにこの木を評価し続ける。
- **Lifecycle Node**: `unconfigured → inactive → active`という状態を持つ特殊なノード。
  `zoh_wrench_planner_component`と`emergencyNode`がこれ。BT側の`LifecycleManager`ノードが
  `configure`/`activate`/`deactivate`を`change_state`サービス経由で明示的に呼ぶまで、
  実際の動作(推力計算・緊急浮上)を開始しない。
- **`WrenchPlan`(`planner_msgs/msg/WrenchPlan`)**: 「目標座標(x, y, z, roll, yaw)」を表す
  メッセージ。`targets`が目標(絶対座標)、`master`/`slave`が現在の位置/速度(省略時は
  `wrench_planner`が`odom`から自動で埋める)。**`targets`は自動で埋まらない**ので、
  目標のうち動かしたくない軸(例: 深度だけ変えてx,yはそのまま)は、送信側が現在値を
  明示的にコピーして送る必要がある([`GoToDepth`](include/bluerov_control_bt_nodes/go_to_depth.hpp)参照)。
- **`zero_order_hold`(ZOH)**: `WrenchPlan`の中継役。`zoh_wrench_plan`で受けた目標を
  `wrench_planner`向けの`goal_current_odom`へそのまま流すが、**送信元(音響班のノード等)からの
  `zoh_wrench_plan`が`timeout_ms`(既定1000ms)以上途絶えると、その瞬間の現在姿勢を目標として
  ホバリング(その場保持)に切り替える**。DVL/odomではなく、あくまで上流の`WrenchPlan`送信の
  途絶を見ている点に注意。
- **`wrench_planner`**: `goal_current_odom`(目標)と`odom`(現在値)からx,y,z,roll,yawの5軸
  独立PIDで力・トルクを計算し、`robot_force`(`geometry_msgs/WrenchStamped`)としてpublishする。
  BlueROVは`mavlink_driver`がこれを受けてPWMへ変換する(`axis.forward/lateral/heave/roll/yaw`)。

## 4. 新規追加したBT葉ノード

| ノード | 役割 | ソース |
|---|---|---|
| `GoToDepth` | 絶対深度(`depth_absolute_m`)または起動時点からの相対オフセット(`depth_offset_m`)へ向かう。x,y,roll,yawはodomから読んだ起動時点の現在値を維持する。深度が`tolerance_m`(既定0.1m)以内になるまで`RUNNING`を返し続け、到達で`SUCCESS`。両ポートとも省略時はROSパラメータ`default_depth_m`(`depth_target_m`launch引数)にフォールバック(DESCEND_TO_7M用) | [go_to_depth.hpp](include/bluerov_control_bt_nodes/go_to_depth.hpp) / [.cpp](src/go_to_depth.cpp) |
| `GoToBlackboardTarget` | BTブラックボードのキー(`target_x`/`target_y`/`target_z`、通常は`{discovered_target...}`のような形で他ノードの出力をバインドする)が示す絶対座標へ向かう。roll/yawは起動時点の現在値を維持。到達判定は`position_tolerance_m`(既定0.2m)/`depth_tolerance_m`(既定0.1m)。VISUAL_ATTACK/HYDRO_APPROACH用。**`mission_origin_x`/`mission_origin_y`/`field_size_m`によるフィールド境界チェック付き**(範囲外なら`publish`せず`FAILURE`。§6参照) | [go_to_blackboard_target.hpp](include/bluerov_control_bt_nodes/go_to_blackboard_target.hpp) / [.cpp](src/go_to_blackboard_target.cpp) |
| `CheckPingerFound` | `pinger_direction`と`zoh_wrench_plan`を購読し、ハイドロフォンで「発見」したかを判定する(旧`handle_search_hydrophone`の発見判定部分)。**移動自体はこのノードの責務ではない**(sbl_controller_nodeが`zoh_wrench_plan`へ発行し続ける値にZOH経由でそのまま追従する)。発見の瞬間、その時点の`zoh_wrench_plan.targets`を`target_x/y/z`へ出力して`SUCCESS`。閾値未達・未受信の間は`RUNNING` | [check_pinger_found.hpp](include/bluerov_control_bt_nodes/check_pinger_found.hpp) / [.cpp](src/check_pinger_found.cpp) |
| `CheckBuoyDetected` | `buoy_detection`を購読し、confidence/timestampで有効性判定(旧`perception.is_buoy_detection_usable`と同じ)。有効ならodomの姿勢で機体座標系オフセットをNEDへ回転し(旧`perception.body_offset_to_ned`相当、tf2で実装)、`target_x/y/z`へ出力して`SUCCESS`。それ以外は`RUNNING`。VISUAL_DETECT用 | [check_buoy_detected.hpp](include/bluerov_control_bt_nodes/check_buoy_detected.hpp) / [.cpp](src/check_buoy_detected.cpp) |
| `CheckRosBoolParam` | ROSパラメータ(`param_name`)の真偽値で即座に`SUCCESS`/`FAILURE`を返す汎用Condition。旧`debug.image_processing_available`/`debug.mic_data_from_above`の2用途に共通で使う | [check_ros_bool_param.hpp](include/bluerov_control_bt_nodes/check_ros_bool_param.hpp) / [.cpp](src/check_ros_bool_param.cpp) |
| `CaptureMissionOrigin` | `odom`を購読し、`status.dvl.id == NORMAL`のodomを受信するまで`RUNNING`で待機する(単に最初のodomではなく、Part Aの`has_valid_measurement_`と同じ考え方でDVLが有効ロックしたことを確認する)。条件を満たしたodomの位置(x, y)を`mission_origin_x`/`mission_origin_y`としてブラックボードへ確定し`SUCCESS`。`GoToBlackboardTarget`のフィールド境界チェックの原点として使われる。個別のタイムアウトは持たない(呼び出し側XMLで`Timeout`に包む。§6参照) | [capture_mission_origin.hpp](include/bluerov_control_bt_nodes/capture_mission_origin.hpp) / [.cpp](src/capture_mission_origin.cpp) |

いずれも「未受信・未確定の間は`RUNNING`のまま待機し、誤って`SUCCESS`を返さない」という
安全側の設計はPhase 1の`GoToDepth`(旧`SetDepthTarget`)と同じ。

これらのノードは`behavior_tree`パッケージの`bt_executor.cpp`に追記して登録している
(各ノードにつき`#include`1行+`factory.registerNodeType<...>`1行。既存ノードの登録・動作には
触れていない)。加えてPhase 2では、状態ごとの個別テスト(§8)のために`bt_executor.cpp`へ
`main_tree_id`という新規ROSパラメータも追加した(空なら従来通りXML自身の
`main_tree_to_execute`属性を使うので、Phase 1時点の挙動に影響はない)。共有パッケージへの
変更なので、取り込む際はメンテナ(Kyubic側)にも一言共有すること。

## 5. `flow.py`の17状態との対応(Phase 2)

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

## 6. ミッション原点とフィールド境界制限(`CaptureMissionOrigin` / `GoToBlackboardTarget`)

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

## 7. 既知の制約・TODO

- **Part A/B/Cのプール再検証が未完了**(冒頭の注意参照)。Phase 2の実機テストはこれを
  先に済ませてから行うこと
- **`CaptureMissionOrigin`の浅水深DVLロックが未検証**(冒頭の注意・§6参照)。Phase 2の
  実機テストはこれも先に確認してから行うこと
- `BatteryCheck`/`CheckSensorsStatus`は未組み込みのまま。Kyubicの閾値(4S/6S電池パック前提)が
  BlueROVにそのまま使えるか未検証のため
- PIDゲイン([config/p_pid_controller_gain_bluerov.yaml](config/p_pid_controller_gain_bluerov.yaml))は
  Phase 1のまま変更していない。実機未チューニングの保守的な初期値。特にroll軸は旧
  `bluerov_control`が一度も制御したことがない軸なので出力レンジを大きく絞ってある
- `sbl_controller_node`(音響班)・画像処理ノードは引き続き外部/未実装。`CheckPingerFound`/
  `CheckBuoyDetected`はそれぞれの入力トピックが来ない限り`RUNNING`のまま安全に待機し続ける
  (§8の`ros2 topic pub`で代用してテストする)
- `RETURN_HOME`は旧実装も無条件成功のスタブだったため、`Surface`到達=ツリー終端という形に
  そのまま対応させた(`planner_msgs/action/Return.action`という定義はあるが、呼び出し元・
  サーバーどちらもKyubic側含めリポジトリ内に存在しないため、ちゃんとした実装はまだ無い)
- `GoToDepth`/`GoToBlackboardTarget`/`CheckBuoyDetected`/`CaptureMissionOrigin`自体は
  個別のタイムアウトを持たない(旧`flow.py`もこれらの移動系状態には個別タイムアウトが
  無かったため、既存の安全水準は維持している)。search_timeout(7分)・
  mic_confirm_timeout(45秒)・AutoSequence全体の大枠タイムアウト
  (`overall_mission_timeout_sec`、既定500秒、旧`flow.py`には無かった新規の安全策)で
  `GoToDepth`/`GoToBlackboardTarget`/`CheckBuoyDetected`を束縛しているが、唯一
  `DESCEND_TO_7M`相当(`Auto`冒頭の`GoToDepth`)はこの500秒タイムアウトの内側にいるため
  最終的には束縛されるものの、個別のタイムアウトは無い。`CaptureMissionOrigin`は
  `Auto`の外(`overall_mission_timeout_ms`の対象外)にいるため、別途
  `origin_capture_timeout_sec`(既定60秒)で束縛している(§6参照)
- 旧`bluerov_control`(`flow.py`/`control.py`)は、このBT構成が実機で一通り検証できてから
  正式に廃止する予定(それまでは両方の起動経路が残る)
- **odom鮮度の監視が無い**: `ZeroOrderHold`は`zoh_wrench_plan`(目標)側の途絶しか監視しておらず、
  `odom`(現在値)側の鮮度は誰も見ていない。§9のPart A修正でDVL/depth/IMUがERROR時に直前値を
  保持するようになった結果、これらのセンサが長時間沈黙し続けると、`odom`が同じ値のまま
  「凍りついた」状態がそれと分からず流れ続けるという新たなリスクがある。対策案
  (`localization_component`が各センサのERROR継続時間を計測し閾値超過でさらなるフラグを立てる、
  `wrench_planner`側で`odom.header.stamp`の鮮度をチェックする等)は未実装、次回以降の検討事項。

## 8. プールテストの進め方

### 8.1 個別テスト方法(`main_tree`launch引数)

`bluerov_phase2.xml`は1ファイルに全SubTreeをまとめてあり、`bt_executor`の`main_tree_id`
ROSパラメータ(`main_tree`launch引数)で、`MainTree`(通し実行)以外のBehaviorTree IDを
直接rootとして起動できる。11mプールでいきなり通しミッションを試すのではなく、まず
状態のかたまりごとに単体で確認すること。

```bash
# ハイドロフォン発見判定だけを単体テスト(移動はしない。CheckPingerFoundは判定専任)
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=MainTree_SearchOnly

# VisualPath(VISUAL_DIVE〜VISUAL_POST_CHECKループ)だけを単体テスト
# (discovered_targetのx/yはCaptureMissionOriginが確定したmission_origin_x/yそのもの、
#  深度はダミー1.0mがXML内でSetBlackboardされる。境界チェックのFAILUREを意図的に
#  試したい場合はbluerov_phase2.xmlのSetBlackboard value属性を書き換えること)
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=MainTree_VisualPath

# HydroPath(HOLD_TARGET〜HYDRO_CHECK_MICループ)だけを単体テスト
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=MainTree_HydroPath

# Surface(SURFACE〜DONE)だけを単体テスト
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=MainTree_Surface
```

`sbl_controller_node`/画像処理ノードがまだ無いので、`CheckPingerFound`/`CheckBuoyDetected`を
動かすには別ターミナルでダミーデータを注入する(機体を拘束した状態で):

```bash
ros2 topic pub /pinger_direction planner_msgs/msg/PingerDirection \
  "{yaw: 0.0, pitch: 35.0, score: 0.01}" --rate 5

ros2 topic pub /perception/buoy_detection bluerov_control_msgs/msg/BuoyDetection \
  "{detected: true, relative_buoy_x_m: 1.0, relative_buoy_y_m: 0.0, relative_buoy_z_m: 0.5, confidence: 0.9}" --rate 5

# sbl_controller_node相当(SEARCH_HYDROPHONE中の追従目標)
ros2 topic pub /planner/wrench_planner/zoh_wrench_plan planner_msgs/msg/WrenchPlan \
  "{z_mode: 0, has_master: true, targets: {x: 1.0, y: 0.0, z: 1.0}}" --rate 5
```

### 8.2 次回プールテストの確認項目: `CaptureMissionOrigin`のDVLロック(§6の未検証リスク)

§6で述べた通り、`CaptureMissionOrigin`は投入直後(競技規則上ひざ程度の浅い水深)で
実行される設計であり、この水深でDVLがビームロックできるかは未検証。**実機テスト前に
必ず以下を浅いプールで計測・確認すること**:

- 投入直後、`ros2 topic echo /localization/odom --field status.dvl.id`等で
  `status.dvl.id`が`NORMAL`(`0`)になるかを確認する
- `NORMAL`になるまでに何秒かかるかを計測し、`origin_capture_timeout_sec`(既定60秒)に
  対して十分な余裕があるかを確認する
- ロックできない、または時間がかかりすぎる場合は、`CaptureMissionOrigin`の呼び出し位置を
  `DescendTo7m`後へ移す設計変更が必要になる(§6参照、現時点では未対応)

### 8.3 初期潜航と緊急浮上を別々に確認する

`depth_target_m`(既定`1.0`m)を競技用水槽の想定深度からプールの実測水深に変える必要があるため、
「潜行できるか」と「深いところからの緊急浮上を再現できるか」を**同じ1回のテストで両立させようと
しないこと**。以下のように分離して確認する。

```bash
# シナリオ1: 初期潜航の確認(プールの実測水深に合わせる)
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py depth_target_m:=1.0
```

```bash
# シナリオ2: 緊急浮上だけを単体で再現する(深い深度に到達させる必要が無い)
# bluerov_bt.launch.py起動後、別ターミナルでemergencyNodeのLifecycle状態を直接操作する。
# BT側のLifecycleManagerが内部でやっているのと同じ操作(change_stateサービス呼び出し)を
# CLIから直接呼ぶだけ(新規コード不要)。ros2 lifecycle set はその定型ラッパーで、
# 生のサービス呼び出し(ros2 service call .../change_state lifecycle_msgs/srv/ChangeState
# "{transition: {id: ...}}")より短く打てる。
ros2 lifecycle set /emergencyNode configure
ros2 lifecycle set /emergencyNode activate
```

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
