# bluerov_control_bt_nodes

BlueROVをKyubicと同じ **Behavior Tree(BT) + `wrench_planner`(P-PID) + `zero_order_hold`(ZOH) +
`emergency`** 構成で動かすための、BlueROV固有の追加分。**Phase 1(土台のみ)**:
「潜行 → (水平方向はsbl_controller_node(音響班、未実装)からのWrenchPlanに追従) → 緊急浮上」
の最小構成で配線を通すことがゴール。VISUAL/HYDRO探索・マイク確認リトライ等、旧
`bluerov_control`の17状態FSMの複雑な分岐はPhase 2以降で扱う。

対象読者: ROS2に触れ始めたばかりの開発者。

## 1. なぜこの構成にしたか

旧`bluerov_control`(`kyubic_ws/src/control/automatic/bluerov_control/`)は状態遷移(`flow.py`)と
カスケードPID(`control.py`)が1つのPythonノードに同居する自前実装だった。Kyubicの`main`ブランチは
同じ役割を4つの独立ノードに分離しており、**実機で実績がある**。調査の結果、
`behavior_tree`/`wrench_planner`/`zero_order_hold`/`emergency`はいずれもKyubic固有ハードウェアへの
依存が無く(`package.xml`にドライバ系パッケージへの依存が無い)、`emergency`は既に今の
`bluerov_control`がソース変更なしで再利用できている。この根拠から、**Kyubicの資産をソース変更なしで
再利用し、BlueROV固有の新規部分だけをこのパッケージ(C++)とBT XMLで書く**方針にした。

旧`bluerov_control`(`flow.py`/`control.py`/`node.py`)は**削除していない**。Phase 1完了時点では
新旧2つの起動経路が並存する(下記2参照)。

## 2. 起動方法

```bash
# 1. BlueROVのハードウェアドライバ
ros2 launch driver_launcher blue_rov_driver.launch.py

# 2. センサ融合(位置・姿勢推定)
ros2 launch localization localization_components.launch.py

# 3a. 【Phase 1: BT構成】このパッケージ
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py

# 3b. 【旧構成、切り戻し用】従来のFSM+PID(3aとは同時に起動しないこと。両方がrobot_forceを
#     送ろうとして競合する)
# ros2 launch bluerov_control bluerov_control.launch.py
```

## 3. このコードとROS2の概念(初心者向け)

- **Behavior Tree(BT)**: 「今何をすべきか」を木構造のXMLで表現する仕組み。ノード(木の葉)は
  `SUCCESS`/`FAILURE`/`RUNNING`のいずれかを返し、`Sequence`(全部成功するまで順に実行)や
  `Fallback`(どれか1つ成功するまで順に試す)といった制御ノードでつなぐ。
  [bt_xml/bluerov_phase1.xml](bt_xml/bluerov_phase1.xml)がその実体。`bt_executor`
  (`behavior_tree`パッケージ、ソース未変更)が10msごとにこの木を評価し続ける。
- **Lifecycle Node**: `unconfigured → inactive → active`という状態を持つ特殊なノード。
  `zoh_wrench_planner_component`と`emergencyNode`がこれ。BT側の`LifecycleManager`ノードが
  `configure`/`activate`/`deactivate`を`change_state`サービス経由で明示的に呼ぶまで、
  実際の動作(推力計算・緊急浮上)を開始しない。
- **`WrenchPlan`(`planner_msgs/msg/WrenchPlan`)**: 「目標座標(x, y, z, roll, yaw)」を表す
  メッセージ。`targets`が目標(絶対座標)、`master`/`slave`が現在の位置/速度(省略時は
  `wrench_planner`が`odom`から自動で埋める)。**`targets`は自動で埋まらない**ので、
  目標のうち動かしたくない軸(例: 深度だけ変えてx,yはそのまま)は、送信側が現在値を
  明示的にコピーして送る必要がある([`SetDepthTarget`](include/bluerov_control_bt_nodes/set_depth_target.hpp)参照)。
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
| `SetDepthTarget` | 起動時に1回だけ、絶対深度目標(`depth_m`)を`zoh_wrench_plan`へpublishする。x,y,roll,yawはodomから読んだ現在値をそのままコピーする(深度だけを変える)。odomを受信するまでは`RUNNING`のまま待機する(安全側) | [set_depth_target.hpp](include/bluerov_control_bt_nodes/set_depth_target.hpp) / [.cpp](src/set_depth_target.cpp) |

このノードは`behavior_tree`パッケージの`bt_executor.cpp`に**2箇所だけ**追記して登録している
(`#include`と`factory.registerNodeType<...>`)。既存ノードの登録・動作には触れていない。
共有パッケージへの変更なので、取り込む際はメンテナ(Kyubic側)にも一言共有すること。

## 5. `flow.py`の17状態との対応(Phase 1時点)

| 旧`flow.py`の状態 | Phase 1での対応 |
|---|---|
| `INIT` | `Delay 2000ms`(BT XML) |
| `DESCEND_TO_7M` | `SetDepthTarget depth_m="7.0"`(x,yは現在地維持のまま深度のみ絶対値指定、旧`_make_absolute_depth_dive_handler`と同じ考え方) |
| `SEARCH_HYDROPHONE`(移動部分のみ) | 未実装。`sbl_controller_node`(音響班)が`zoh_wrench_plan`へ発行し続けるWrenchPlanにZOH経由でそのまま追従する想定(「発見」判定・VISUAL/HYDRO分岐は無い) |
| `EMERGENCY` | `Emergency` SubTree(`emergencyNode`をactivate。旧実装の`_enter_emergency`と同じ最終挙動) |
| それ以外(`VISUAL_*`, `HYDRO_*`, `HOLD_TARGET`, `SURFACE`, `RETURN_HOME`, `DONE`) | **Phase 2以降**。特に`RETURN_HOME`相当は`planner_msgs/action/Return.action`という定義はあるがKyubic側にも参考実装が無く、ゼロから設計が必要 |

## 6. 既知の制約・TODO(Phase 2以降)

- `BatteryCheck`/`CheckSensorsStatus`は未組み込み。Kyubicの閾値(4S/6S電池パック前提)が
  BlueROVにそのまま使えるか未検証のため
- PIDゲイン([config/p_pid_controller_gain_bluerov.yaml](config/p_pid_controller_gain_bluerov.yaml))は
  実機未チューニングの保守的な初期値。特にroll軸は旧`bluerov_control`が一度も制御したことが
  ない軸なので出力レンジを大きく絞ってある
- 「発見」判定・VISUAL/HYDRO探索・マイク確認リトライ・SURFACE/RETURN_HOMEはすべて未実装
- 旧`bluerov_control`(`flow.py`/`control.py`)は、Phase 2でこのBT構成に一通り移行できてから
  正式に廃止する予定(それまでは両方の起動経路が残る)
