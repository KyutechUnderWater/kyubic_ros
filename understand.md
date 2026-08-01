# iwakuni2026.xml トピック流れ

`behavior_tree/bt_xml/iwakuni2026.xml` が参照・発行する ROS 2 トピック／アクション／サービスの関係図。
BlueROV 向け起動は `bluerov_control_bt_nodes/launch/bluerov_bt.launch.py` を前提とし、
`btExecutorNode` の remapping を反映している。

## 全体アーキテクチャ

```mermaid
flowchart TB
  subgraph Input["センサ・入力"]
    IMU["/driver/imu<br/>driver_msgs/IMU"]
    DVL["/driver/dvl<br/>driver_msgs/DVL"]
    DEPTH["/driver/depth<br/>driver_msgs/Depth"]
    LEAK["leak<br/>driver_msgs/BoolStamped"]
    POWER["power_state<br/>driver_msgs/PowerState"]
    JOY["/joy_common/joy_common<br/>joy_common_msgs/Joy"]
    PINGER["/pinger_direction<br/>planner_msgs/PingerDirection"]
    BUOY["/buoy/relative_position<br/>buoy_interfaces/BuoyRelativePosition"]
  end

  subgraph Localization["位置推定"]
    LOC["localization fusion"]
    ODOM["/localization/odom<br/>localization_msgs/Odometry"]
    RESET["/localization/reset<br/>サービス"]
  end

  subgraph BT["btExecutorNode (iwakuni2026.xml)"]
    MAIN["MainTree"]
    AUTO["BlueROV2Auto"]
    MANUAL["Manual"]
    EMERG["Emergency"]
  end

  subgraph Planning["経路・制御プランナ"]
    PDLA["pdla_planner<br/>Action: /planner/pdla_planner/pdla_plan"]
    ZOH_PLAN["/planner/wrench_planner/zoh_wrench_plan<br/>planner_msgs/WrenchPlan"]
    ZOH["zero_order_hold"]
    GOAL["/planner/wrench_planner/goal_current_odom<br/>planner_msgs/WrenchPlan"]
    WP["wrench_planner"]
    TARGETS["/planner/wrench_planner/targets"]
  end

  subgraph Actuation["作動"]
    RF["/driver/blue_rov/mavlink_driver/robot_force<br/>geometry_msgs/WrenchStamped"]
    HB["/driver/blue_rov/mavlink_driver/heartbeat<br/>std_msgs/Bool"]
    JOY2W["joy2wrench (manualNode)"]
    EMG_NODE["emergencyNode"]
  end

  subgraph Output["ログ・外部"]
    TALKER["/talker<br/>std_msgs/String"]
    SBL["find_pinger Action<br/>(音響班 sbl_estimation_node 等)"]
  end

  IMU --> LOC
  DVL --> LOC
  DEPTH --> LOC
  LOC --> ODOM

  ODOM --> BT
  ODOM --> PDLA
  ODOM --> ZOH
  ODOM --> WP
  ODOM --> AUTO

  PINGER --> AUTO
  BUOY --> AUTO

  JOY --> MAIN
  POWER --> MAIN
  IMU --> MAIN
  DVL --> MAIN
  DEPTH --> MAIN
  LEAK --> MAIN

  MAIN -->|mode=auto| AUTO
  MAIN -->|mode=manual| MANUAL
  MAIN -->|異常時| EMERG

  AUTO -->|WaypointAction| PDLA
  AUTO -->|FindPingerAction| SBL
  AUTO -->|ResetLocalization| RESET
  AUTO --> TALKER

  PDLA -->|WrenchPlan 出力| ZOH_PLAN
  ZOH_PLAN --> ZOH
  ZOH --> GOAL
  GOAL --> WP
  WP --> RF
  WP --> TARGETS

  JOY2W -->|manual 時| RF
  EMG_NODE -->|emergency 時| RF
  HB --> Actuation

  style BT fill:#e8f4fc
  style Planning fill:#fff4e6
  style Input fill:#f0f0f0
```

## 制御チェーン（Auto モード）

`WaypointAction` が PDLA に CSV を渡すと、目標値が PID 制御まで流れる。

```mermaid
flowchart LR
  CSV["BT: CSV 書き込み<br/>(Write*WaypointCSV 等)"]
  WA["WaypointAction<br/>(BT)"]
  PDLA["pdla_planner<br/>/planner/pdla_planner/pdla_plan"]
  ZWP["/planner/wrench_planner/zoh_wrench_plan"]
  ZOH["zero_order_hold<br/>(Lifecycle activate)"]
  GCO["/planner/wrench_planner/goal_current_odom"]
  WR["wrench_planner<br/>(Lifecycle activate)"]
  RF["/driver/blue_rov/mavlink_driver/robot_force"]
  ODOM["/localization/odom"]

  CSV --> WA
  WA -->|Action Goal: csv_file_path| PDLA
  ODOM --> PDLA
  PDLA -->|WrenchPlan publish| ZWP
  ZWP --> ZOH
  ODOM --> ZOH
  ZOH --> GCO
  GCO --> WR
  ODOM --> WR
  WR --> RF
```

## MainTree（モード切替）

```mermaid
flowchart TB
  subgraph MainTree["MainTree"]
    BC["BatteryCheck"]
    CSS["CheckSensorsStatus"]
    UM["UpdateMode"]
    SW{"Switch2<br/>mode"}
    CFG["Configure SubTree"]
    MAN["Manual SubTree"]
    AUT["BlueROV2Auto SubTree"]
    EMG["Emergency SubTree"]
  end

  POWER["power_state"] -->|sub| BC
  IMU["/driver/imu"] -->|sub| CSS
  DEPTH["/driver/depth"] -->|sub| CSS
  DVL["/driver/dvl"] -->|sub| CSS
  LEAK["leak"] -->|sub| CSS
  JOY["/joy_common/joy_common"] -->|sub| UM

  BC -->|FAILURE| EMG
  CFG --> CSS --> UM --> SW
  SW -->|manual / circle| MAN
  SW -->|auto / cross| AUT
  SW -->|その他| EMG

  MAN -->|activate| J2W["joy2wrench<br/>→ robot_force"]
  AUT -->|activate| ZOH_WP["wrench_planner + ZOH"]
  EMG -->|deactivate ZOH<br/>activate emergency| EMG_RF["emergency<br/>→ robot_force"]

  MAN --> TALK["/talker"]
  AUT --> TALK
  EMG --> TALK
```

## BlueROV2Auto フェーズ別トピック

ミッション進行に沿った購読・発行の対応。

```mermaid
flowchart TB
  subgraph Phase0["起動・初期化"]
    T0["Talker → /talker"]
    R0["ResetLocalization → /localization/reset"]
    L0["LifecycleManager → wrench_planner activate"]
  end

  subgraph Phase1["浅い遷移・深潜"]
    W1["WaypointAction → pdla_plan"]
    CSV1["transit_shallow.csv / descend_deep.csv"]
    CSV1 --> W1
  end

  subgraph Phase2["ハイドロフォン追跡ループ"]
  direction TB
    CPP["CheckPingerPitch"]
    WH["WriteHydrophoneWaypointCSV"]
    W2["WaypointAction"]
    CPP -->|pitch ≤ 50°| EXIT2["ループ脱出"]
    CPP -->|まだ遠い| WH --> W2 --> RETRY2["AlwaysFailure → 再試行"]
    P2["/pinger_direction"] --> CPP
    P2 --> WH
    O2["/localization/odom"] --> WH
  end

  subgraph Phase3["最終降下"]
    WAO["WriteAxisOverrideWaypointCSV<br/>(z=2.5 altitude)"]
    W3["WaypointAction"]
    O3["/localization/odom"] --> WAO --> W3
  end

  subgraph Phase4["Find Pinger"]
    FP["FindPingerAction → find_pinger"]
  end

  subgraph Phase5["ビジョン接近ループ (180s)"]
    CBP["CheckBuoyPosition"]
    WVAO["WriteAxisOverrideWaypointCSV"]
    WVV["WriteVisionWaypointCSV"]
    CBI["CheckBuoyInFrame"]
    W5["WaypointAction"]
    B5["/buoy/relative_position"] --> CBP
    B5 --> WVV
    B5 --> CBI
    O5["/localization/odom"] --> CBP
    O5 --> WVAO
    O5 --> WVV
    CBP -->|接触判定| WVAO --> W5
    WVV --> W5
  end

  subgraph Phase6["解放確認ループ (30s)"]
    CPP6["CheckPingerPitch (pitch > 70°)"]
    CBI6["CheckBuoyInFrame (未検出)"]
    P6["/pinger_direction"] --> CPP6
    B6["/buoy/relative_position"] --> CBI6
  end

  subgraph Phase7["帰還"]
    R1["WriteAxisOverrideWaypointCSV (x=11)"]
    W7a["WaypointAction → return_step1.csv"]
    W7b["WaypointAction → return_final.csv"]
    O7["/localization/odom"] --> R1 --> W7a --> W7b
  end

  Phase0 --> Phase1 --> Phase2 --> Phase3 --> Phase4 --> Phase5 --> Phase6 --> Phase7
```

## トピック一覧（XML 参照分）

| トピック / インターフェース | 型 | 方向 (BT から見て) | 使用ノード |
|---|---|---|---|
| `/talker` | `std_msgs/String` | pub | `Talker` (各フェーズの状態メッセージ) |
| `/localization/reset` | サービス | call | `ResetLocalization` |
| `/localization/odom` | `localization_msgs/Odometry` | sub | `Write*WaypointCSV`, `CheckBuoyPosition` 等 (`odom` → remap) |
| `/pinger_direction` | `planner_msgs/PingerDirection` | sub | `CheckPingerPitch`, `WriteHydrophoneWaypointCSV` |
| `/buoy/relative_position` | `buoy_interfaces/BuoyRelativePosition` | sub | `CheckBuoyPosition`, `WriteVisionWaypointCSV`, `CheckBuoyInFrame` |
| `/joy_common/joy_common` | `joy_common_msgs/Joy` | sub | `UpdateMode` (circle=manual, cross=auto) |
| `power_state` | `driver_msgs/PowerState` | sub | `BatteryCheck` |
| `imu` / `depth` / `dvl` / `leak` | 各 driver_msgs | sub | `CheckSensorsStatus` |
| `/planner/pdla_planner/pdla_plan` | Action (`planner_msgs/PDLA`) | client | `WaypointAction` |
| `find_pinger` | Action (`planner_msgs/FindPinger`) | client | `FindPingerAction` |
| `/planner/wrench_planner/zoh_wrench_plan` | `planner_msgs/WrenchPlan` | (間接) | PDLA 出力 → ZOH 入力 |
| `/planner/wrench_planner/goal_current_odom` | `planner_msgs/WrenchPlan` | (間接) | ZOH 出力 → wrench_planner 入力 |
| `/driver/blue_rov/mavlink_driver/robot_force` | `geometry_msgs/WrenchStamped` | (間接) | wrench_planner / joy2wrench / emergency 出力 |
| `/driver/blue_rov/mavlink_driver/heartbeat` | `std_msgs/Bool` | (並行 pub) | `bluerov_heartbeat_publisher` (BT 外) |

## 補足

- **前提起動**: `blue_rov_driver.launch.py` + `localization_components.launch.py` + `planner_launcher.launch.py` (PDLA) + `bluerov_bt.launch.py`。
- **`/buoy/relative_position`**: `perception/buoy_detector` が直接発行。`buoy_detector_driver` が `/perception/buoy_detection` へ変換するが、iwakuni2026.xml は生の `/buoy/relative_position` を参照する。
- **`find_pinger`**: 音響班の `sbl_estimation_node` 等が提供する Action。BT ノード名は相対の `find_pinger`（フルパスは起動時の namespace に依存）。
- **Lifecycle**: Auto 開始時に ZOH + wrench_planner を `activate`、Manual 時は `manualNode`(joy2wrench)、Emergency 時は ZOH を `deactivate` して `emergencyNode` を `activate`。
