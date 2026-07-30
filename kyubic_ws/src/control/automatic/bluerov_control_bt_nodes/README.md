# bluerov_control_bt_nodes

BlueROVをKyubicと同じ **Behavior Tree(BT) + `wrench_planner`(P-PID) + `zero_order_hold`(ZOH) +
`emergency`** 構成で動かすための、BlueROV固有の追加分。

**Phase 2(このドキュメントの現状)**: 旧`bluerov_control/flow.py`の17状態すべてをBT XML
(`bt_xml/bluerov_phase2.xml`)へ移行済み。Phase 1(`bluerov_phase1.xml`、固定深度潜航のみ)は
参考として残っているが、`bluerov_bt.launch.py`はPhase 2のXMLを使う。

**重要な注意**: このPhase 2着手時点で、Phase 1のDVL/odomゼロ値伝播バグ修正(§8のPart A/B/C)は
**プールでまだ再検証されていない**。依頼者はこれを把握した上でPhase 2の実装を進めることを
明示的に選択した。実機テストの前に必ずPart A/B/Cの再検証を先に行うこと。

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

# 3a'. 【個別テスト】特定のSubTreeだけを単体起動する(§7参照)
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
| `GoToBlackboardTarget` | BTブラックボードのキー(`target_x`/`target_y`/`target_z`、通常は`{discovered_target...}`のような形で他ノードの出力をバインドする)が示す絶対座標へ向かう。roll/yawは起動時点の現在値を維持。到達判定は`position_tolerance_m`(既定0.2m)/`depth_tolerance_m`(既定0.1m)。VISUAL_ATTACK/HYDRO_APPROACH用 | [go_to_blackboard_target.hpp](include/bluerov_control_bt_nodes/go_to_blackboard_target.hpp) / [.cpp](src/go_to_blackboard_target.cpp) |
| `CheckPingerFound` | `pinger_direction`と`zoh_wrench_plan`を購読し、ハイドロフォンで「発見」したかを判定する(旧`handle_search_hydrophone`の発見判定部分)。**移動自体はこのノードの責務ではない**(sbl_controller_nodeが`zoh_wrench_plan`へ発行し続ける値にZOH経由でそのまま追従する)。発見の瞬間、その時点の`zoh_wrench_plan.targets`を`target_x/y/z`へ出力して`SUCCESS`。閾値未達・未受信の間は`RUNNING` | [check_pinger_found.hpp](include/bluerov_control_bt_nodes/check_pinger_found.hpp) / [.cpp](src/check_pinger_found.cpp) |
| `CheckBuoyDetected` | `buoy_detection`を購読し、confidence/timestampで有効性判定(旧`perception.is_buoy_detection_usable`と同じ)。有効ならodomの姿勢で機体座標系オフセットをNEDへ回転し(旧`perception.body_offset_to_ned`相当、tf2で実装)、`target_x/y/z`へ出力して`SUCCESS`。それ以外は`RUNNING`。VISUAL_DETECT用 | [check_buoy_detected.hpp](include/bluerov_control_bt_nodes/check_buoy_detected.hpp) / [.cpp](src/check_buoy_detected.cpp) |
| `CheckRosBoolParam` | ROSパラメータ(`param_name`)の真偽値で即座に`SUCCESS`/`FAILURE`を返す汎用Condition。旧`debug.image_processing_available`/`debug.mic_data_from_above`の2用途に共通で使う | [check_ros_bool_param.hpp](include/bluerov_control_bt_nodes/check_ros_bool_param.hpp) / [.cpp](src/check_ros_bool_param.cpp) |

いずれも「未受信・未確定の間は`RUNNING`のまま待機し、誤って`SUCCESS`を返さない」という
安全側の設計はPhase 1の`GoToDepth`(旧`SetDepthTarget`)と同じ。

これらのノードは`behavior_tree`パッケージの`bt_executor.cpp`に追記して登録している
(各ノードにつき`#include`1行+`factory.registerNodeType<...>`1行。既存ノードの登録・動作には
触れていない)。加えてPhase 2では、状態ごとの個別テスト(§7)のために`bt_executor.cpp`へ
`main_tree_id`という新規ROSパラメータも追加した(空なら従来通りXML自身の
`main_tree_to_execute`属性を使うので、Phase 1時点の挙動に影響はない)。共有パッケージへの
変更なので、取り込む際はメンテナ(Kyubic側)にも一言共有すること。

## 5. `flow.py`の17状態との対応(Phase 2)

| 旧`flow.py`の状態(群) | BT表現(`bluerov_phase2.xml`) |
|---|---|
| `INIT` | `Delay 2000ms` |
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

`AutoSequence`全体は`Timeout msec=1800000`(30分)で覆ってあり、個々のノードが無限に
`RUNNING`し続けることが無いようにしている(§6参照)。

## 6. 既知の制約・TODO

- **Part A/B/Cのプール再検証が未完了**(冒頭の注意参照)。Phase 2の実機テストはこれを
  先に済ませてから行うこと
- `BatteryCheck`/`CheckSensorsStatus`は未組み込みのまま。Kyubicの閾値(4S/6S電池パック前提)が
  BlueROVにそのまま使えるか未検証のため
- PIDゲイン([config/p_pid_controller_gain_bluerov.yaml](config/p_pid_controller_gain_bluerov.yaml))は
  Phase 1のまま変更していない。実機未チューニングの保守的な初期値。特にroll軸は旧
  `bluerov_control`が一度も制御したことがない軸なので出力レンジを大きく絞ってある
- `sbl_controller_node`(音響班)・画像処理ノードは引き続き外部/未実装。`CheckPingerFound`/
  `CheckBuoyDetected`はそれぞれの入力トピックが来ない限り`RUNNING`のまま安全に待機し続ける
  (§7の`ros2 topic pub`で代用してテストする)
- `RETURN_HOME`は旧実装も無条件成功のスタブだったため、`Surface`到達=ツリー終端という形に
  そのまま対応させた(`planner_msgs/action/Return.action`という定義はあるが、呼び出し元・
  サーバーどちらもKyubic側含めリポジトリ内に存在しないため、ちゃんとした実装はまだ無い)
- `GoToDepth`/`GoToBlackboardTarget`/`CheckBuoyDetected`自体は個別のタイムアウトを持たない
  (旧`flow.py`もこれらの移動系状態には個別タイムアウトが無かったため、既存の安全水準は
  維持している)。search_timeout(7分)・mic_confirm_timeout(45秒)・AutoSequence全体の
  大枠タイムアウト(30分、旧`flow.py`には無かった新規の安全策)でこれらを束縛しているが、
  唯一`DESCEND_TO_7M`相当(`Auto`冒頭の`GoToDepth`)はこの30分タイムアウトの内側にいるため
  最終的には束縛されるものの、個別のタイムアウトは無い
- 旧`bluerov_control`(`flow.py`/`control.py`)は、このBT構成が実機で一通り検証できてから
  正式に廃止する予定(それまでは両方の起動経路が残る)
- **odom鮮度の監視が無い**: `ZeroOrderHold`は`zoh_wrench_plan`(目標)側の途絶しか監視しておらず、
  `odom`(現在値)側の鮮度は誰も見ていない。§8のPart A修正でDVL/depth/IMUがERROR時に直前値を
  保持するようになった結果、これらのセンサが長時間沈黙し続けると、`odom`が同じ値のまま
  「凍りついた」状態がそれと分からず流れ続けるという新たなリスクがある。対策案
  (`localization_component`が各センサのERROR継続時間を計測し閾値超過でさらなるフラグを立てる、
  `wrench_planner`側で`odom.header.stamp`の鮮度をチェックする等)は未実装、次回以降の検討事項。

## 7. プールテストの進め方

### 7.1 個別テスト方法(`main_tree`launch引数)

`bluerov_phase2.xml`は1ファイルに全SubTreeをまとめてあり、`bt_executor`の`main_tree_id`
ROSパラメータ(`main_tree`launch引数)で、`MainTree`(通し実行)以外のBehaviorTree IDを
直接rootとして起動できる。11mプールでいきなり通しミッションを試すのではなく、まず
状態のかたまりごとに単体で確認すること。

```bash
# ハイドロフォン発見判定だけを単体テスト(移動はしない。CheckPingerFoundは判定専任)
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=MainTree_SearchOnly

# VisualPath(VISUAL_DIVE〜VISUAL_POST_CHECKループ)だけを単体テスト
# (discovered_targetはダミー座標(0, 0, 1.0m)がXML内でSetBlackboardされる)
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

### 7.2 初期潜航と緊急浮上を別々に確認する

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

## 8. プール実験で発見された不具合の修正(2026年)

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
