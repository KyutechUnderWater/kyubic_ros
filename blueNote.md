# BlueROV ドライバーまとめ

`kyubic_ws/src/driver/blue_rov` は、**BlueROV（ArduSub）** と **Cerulean Sonar DVL-75** を ROS 2 に接続するためのドライバー群です。Kyubic 機体向けドライバーと同じ「共通トピック」設計を採用しており、上層の制御・ローカライゼーションは機体を差し替えても同じトピック名でセンサを読めます。

---

## 1. パッケージ構成

```
blue_rov/
├── mavlink_driver/     # BlueROV ↔ MAVLink（操縦・テレメトリ）
├── dvl75_driver/       # DVL-75 ↔ UDP（速度・高度）
└── blue_rov_msgs/      # DVL-75 専用メッセージ定義
```

| パッケージ | 言語 | 役割 |
|---|---|---|
| `mavlink_driver` | Python | MAVLink 接続の**単独所有者**。RC override で操縦、テレメトリを共通メッセージへ変換 |
| `dvl75_driver` | C++ | DVL-75 の UDP/NMEA をパースし、共通 `DVL` と専用 `DVL75` を Publish |
| `blue_rov_msgs` | msg 定義 | 機体共通で使わない DVL-75 固有フィールドだけを定義 |

---

## 2. 設計思想

### 2.1 ドライバーが通信を独占する

- **MAVLink 接続は `mavlink_driver` だけが持つ**。別プロセスから RC override を送ると競合する（README にも明記）。
- 制御プログラムは **ROS トピック**（`robot_force` など）で指令を出し、ドライバーが MAVLink に変換する。

### 2.2 共通トピックと機体固有トピックの分離

| 種別 | 例 | 意図 |
|---|---|---|
| **機体に依存しない共通トピック** | `/driver/imu`, `/driver/depth`, `/driver/dvl` | localization・dashboard など上層は機体を意識せず Subscribe できる |
| **BlueROV 固有トピック** | `/driver/blue_rov/mavlink_driver/vehicle_state` など | MAVLink 固有の状態・操縦入出力は namespace 配下に置く |
| **デバイス固有メッセージ** | `blue_rov_msgs/DVL75` | 共通 `driver_msgs/DVL` の互換性を壊さず DVL-75 詳細を載せる |

Kyubic では `/driver/actuator_rp2040_driver/robot_force` に remap していた制御系は、BlueROV では `/driver/blue_rov/mavlink_driver/robot_force` に向ける必要があります。センサ系（`/driver/dvl` 等）は **remap 不要** です。

### 2.3 安全設計（mavlink_driver）

RC override が実際に機体へ届くのは、次を**すべて**満たすときだけです。

1. MAVLink で機体と接続済み
2. 機体が **ARM 済み**（自動 ARM はしない。`set_armed` サービスで明示的に行う）
3. `robot_force` が `command_timeout_s`（既定 0.5 s）以内に更新されている
4. `heartbeat`（`std_msgs/Bool`）が `control_heartbeat_timeout_s`（既定 1.0 s）以内に `true` で届いている（`require_control_heartbeat: true` の場合）

条件を満たさないときは中立 PWM を送り、推力を出さない。

---

## 3. データの流れ（全体像）

```mermaid
flowchart TB
    subgraph 物理層
        BR[BlueROV / ArduSub<br/>MAVLink UDP :14550]
        DVL[Cerulean DVL-75<br/>UDP :27000]
    end

    subgraph blue_rov ドライバー
        MD[mavlink_driver<br/>Python]
        DD[dvl75_driver<br/>C++]
    end

    subgraph 共通トピック
        T_IMU["/driver/imu"]
        T_DEPTH["/driver/depth"]
        T_DVL["/driver/dvl"]
    end

    subgraph BlueROV 固有
        T_RF["/driver/blue_rov/mavlink_driver/robot_force"]
        T_HB["/driver/blue_rov/mavlink_driver/heartbeat"]
        T_VS["/driver/blue_rov/mavlink_driver/vehicle_state"]
        T_DVL75["/driver/blue_rov/dvl75_driver/dvl75"]
        S_ARM["/driver/blue_rov/mavlink_driver/set_armed"]
    end

    subgraph 制御・上層（例）
        CTRL[joy2wrench / wrench_planner / p_pid など]
        LOC[localization]
        DASH[dashboard]
    end

    BR <-->|MAVLink| MD
    DVL -->|UDP NMEA| DD

    MD --> T_IMU & T_DEPTH & T_VS
    MD <-- T_RF & T_HB
    MD --- S_ARM

    DD --> T_DVL & T_DVL75

    CTRL -->|WrenchStamped| T_RF
    CTRL -->|Bool heartbeat| T_HB
    LOC & DASH --> T_IMU & T_DEPTH & T_DVL
```

### 3.1 mavlink_driver：入出力一覧

**Subscribe（指令の入力）** — namespace: `/driver/blue_rov/mavlink_driver`

| 相対名 | 型 | 内容 |
|---|---|---|
| `robot_force` | `geometry_msgs/WrenchStamped` | 操縦指令（下記の軸対応） |
| `heartbeat` | `std_msgs/Bool` | 制御プログラムの生存確認（`true`） |
| `led` | `driver_msgs/LED` | 左右 LED の PWM |
| `camera_tilt` | `driver_msgs/Int32Stamped` | カメラチルト角度 [deg] |

`robot_force` の軸対応:

| Wrench フィールド | 軸 | 既定 RC ch |
|---|---|---|
| `force.x` | forward（前後） | 5 |
| `force.y` | lateral（左右） | 6 |
| `force.z` | heave（上下） | 3 |
| `torque.x` | roll | 2 |
| `torque.z` | yaw | 4 |

各軸の値は `axis.<name>.limit` で正規化され PWM（1100–1900、中立 1500）に変換されます。符号は `axis.<name>.invert` で反転可能。

**Publish（センサ・状態の出力）**

| トピック | 型 | 備考 |
|---|---|---|
| `/driver/imu` | `driver_msgs/IMU` | MAVLink `ATTITUDE` 由来（姿勢・角速度） |
| `/driver/depth` | `driver_msgs/Depth` | `VFR_HUD` + `SCALED_PRESSURE2`（水温） |
| `.../power_state` | `driver_msgs/PowerState` | バッテリー電圧・電流 |
| `.../gnss` | `driver_msgs/Gnss` | GPS（水上時など） |
| `.../vehicle_state` | `driver_msgs/VehicleState` | 接続・ARM 状態（0.2 s 周期） |

**Service**

| 名前 | 型 | 内容 |
|---|---|---|
| `.../set_armed` | `std_srvs/SetBool` | `true`=ARM, `false`=DISARM（ACK + heartbeat で確認） |

### 3.2 dvl75_driver：入出力一覧

**Publish**

| トピック | 型 | 内容 |
|---|---|---|
| `/driver/dvl` | `driver_msgs/DVL` | 速度・高度・有効フラグ（Pathfinder と共通） |
| `.../dvl75` | `blue_rov_msgs/DVL75` | ビーム状態・DVL 内部時刻・位置/角度差分など |

**Service**

| 名前 | 型 | 内容 |
|---|---|---|
| `.../command` | `driver_msgs/Command` | DVL へ 1 行 ASCII コマンドを UDP 送信 |

UDP で `$DVEXT`（最新保持）と `$DVPDL`（受信のたびに統合 Publish）を処理します。

---

## 4. 立ち上げ方

### 4.1 ビルド（初回・コード変更後）

```bash
cd /kyubic_ros/kyubic_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to mavlink_driver dvl75_driver blue_rov_msgs --symlink-install
source install/setup.bash
```

### 4.2 通常運用（おすすめ：両ドライバーをまとめて起動）

```bash
ros2 launch driver_launcher blue_rov_driver.launch.py
```

`log_level` を変える場合:

```bash
ros2 launch driver_launcher blue_rov_driver.launch.py log_level:=debug
```

### 4.3 個別起動

```bash
# MAVLink のみ
ros2 launch mavlink_driver mavlink_driver.launch.py

# DVL-75 のみ（通常運用）
ros2 launch dvl75_driver dvl75_driver.launch.py
```

### 4.4 DVL-75 初回セットアップ（1 回だけ）

DVL を初めてネットワークに繋ぐときだけ:

```bash
ros2 launch dvl75_driver dvl75_setup.launch.py
```

- `SEND-DVEXT ON` / `SEND-DVPDL ON` を DVL に永続設定
- `$DVPDL` が来ることを確認したら `Ctrl-C`
- **通常 launch と同時起動しない**
- 以降は `dvl75_driver.launch.py` のみ

### 4.5 起動前のネットワーク設定

| デバイス | 接続 | 既定設定 |
|---|---|---|
| BlueROV | MAVLink UDP | `udpin:0.0.0.0:14550`（`mavlink_driver.param.yaml` の `endpoint`） |
| DVL-75 | UDP 受信 | `bind 0.0.0.0:27000`、`dvl_address: 192.168.2.3` |

DVL の送信元 IP が違う場合は `dvl75_driver/config/dvl75_driver.param.yaml` の `dvl_address` を実機 IP に合わせる。

### 4.6 ARM（推力を出す前に必須）

```bash
ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool "{data: true}"
```

停止・安全確認:

```bash
ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool "{data: false}"
```

### 4.7 動作確認コマンド

```bash
# ノード一覧
ros2 node list

# 接続状態
ros2 topic echo /driver/blue_rov/mavlink_driver/vehicle_state

# センサ
ros2 topic echo /driver/imu
ros2 topic echo /driver/depth
ros2 topic hz /driver/dvl

# ノードの入出力詳細
ros2 node info /driver/blue_rov/mavlink_driver/mavlink_driver
ros2 node info /driver/blue_rov/dvl75_driver/dvl75_driver
```

停止は launch 端末で `Ctrl-C`。mavlink_driver は終了時に中立 RC を送る。

---

## 5. 設定ファイルの場所

| ファイル | 内容 |
|---|---|
| `mavlink_driver/config/mavlink_driver.param.yaml` | MAVLink 接続先、PWM 範囲、RC チャンネル、安全タイムアウト |
| `dvl75_driver/config/dvl75_driver.param.yaml` | DVL IP、UDP ポート、軸符号、信頼度しきい値 |

launch ファイルがこれらを読み込み、**共通トピックへの remap** もここで行います。

- `mavlink_driver/launch/mavlink_driver.launch.py`  
  `imu` → `/driver/imu`、`depth` → `/driver/depth`
- `dvl75_driver/launch/dvl75_driver.launch.py`  
  `dvl` → `/driver/dvl`

---

## 6. コードの読み方（かみ砕き）

### 6.1 mavlink_driver（Python）

| ファイル | 何をしているか |
|---|---|
| `mavlink_driver/mavlink_driver.py` | メインノード。MAVLink 受信スレッド + ROS タイマーで RC 送信・状態 Publish |
| `mavlink_driver/conversions.py` | 物理指令 → PWM、オイラー角 → クォータニオンなど純粋な変換関数 |
| `launch/mavlink_driver.launch.py` | ノード起動、namespace、remap、パラメータ YAML 読込 |
| `config/mavlink_driver.param.yaml` | 実機向けチューニング値 |

**処理の流れ（mavlink_driver.py）**

1. `__init__` でパラメータ読込・ROS の pub/sub/srv 作成
2. `_receiver_loop`（別スレッド）で MAVLink 接続・heartbeat 監視・テレメトリ受信
3. `robot_force` コールバックで最新の軸指令を保持
4. `_send_override_timer`（10 Hz 既定）で条件を満たせば RC override 送信
5. 終了時 `destroy_node` で中立 PWM を送信

### 6.2 dvl75_driver（C++）

| ファイル | 何をしているか |
|---|---|
| `include/dvl75_driver/dvl75_driver.hpp` | ノードクラス宣言 |
| `src/dvl75_driver.cpp` | UDP 受信スレッド、Publish、コマンドサービス |
| `src/dvl75_parser.cpp` | `$DVEXT` / `$DVPDL` の NMEA パース |
| `launch/dvl75_driver.launch.py` | `component_container_mt` に composable ノードをロード |

ROS 2 の **Composable Node** として動き、マルチスレッドコンテナ内で UDP 受信と Publish を行います。

### 6.3 blue_rov_msgs

`msg/DVL75.msg` — DVL-75 だけが持つビーム情報・DVL 内部時刻・位置/角度差分。共通の `driver_msgs/DVL` には載せない。

---

## 7. 他の制御プログラムへの組み込み方

既存の Kyubic 向け制御（`joy2wrench`, `wrench_planner`, `p_pid_controller` など）は、launch で `robot_force` を次のように remap しています。

```python
("robot_force", "/driver/actuator_rp2040_driver/robot_force")
```

BlueROV では **次の 2 点** に remap を変更します。

```python
("robot_force", "/driver/blue_rov/mavlink_driver/robot_force"),
("heartbeat", "/driver/blue_rov/mavlink_driver/heartbeat"),  # 必須（既定設定時）
```

### 7.1 制御ノード側で必要なこと

1. **`geometry_msgs/WrenchStamped` を Publish**  
   - 単位は `axis.*.limit` と対応する物理量（N, N·m など、実機キャリブレーションに依存）
   - `command_timeout_s`（0.5 s）より高い頻度で送る（10 Hz 以上推奨）

2. **`std_msgs/Bool` の heartbeat を Publish**  
   - `data: true` を 1 Hz 以上で送る（制御ループと同じタイマーでよい）
   - 現状リポジトリ内の制御 launch には heartbeat の remap が無いため、**BlueROV 用 launch を追加するか remap を足す必要がある**

3. **ARM は制御ノードの責務ではないが、運用では明示的に行う**  
   - 起動後に `set_armed` サービスを呼ぶか、操縦開始前の手順に含める

4. **センサは共通トピックをそのまま Subscribe**  
   - `/driver/dvl`, `/driver/imu`, `/driver/depth` — localization 等は変更不要

### 7.2 launch 変更例（概念）

```python
Node(
    package="joy2wrench",
    executable="joy2wrench",
    remappings=[
        ("robot_force", "/driver/blue_rov/mavlink_driver/robot_force"),
        ("heartbeat", "/driver/blue_rov/mavlink_driver/heartbeat"),
    ],
)
```

制御ノードが heartbeat をまだ Publish していない場合は、タイマーで `true` を送る小さなノードを足すか、ノード本体に Publisher を追加します。

### 7.3 最小の手動操縦テスト（概念）

```bash
# 1. ドライバー起動
ros2 launch driver_launcher blue_rov_driver.launch.py

# 2. heartbeat（別端末・例）
ros2 topic pub -r 10 /driver/blue_rov/mavlink_driver/heartbeat std_msgs/msg/Bool "{data: true}"

# 3. ARM
ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool "{data: true}"

# 4. 前進指令の例（force.x = forward）
ros2 topic pub -r 10 /driver/blue_rov/mavlink_driver/robot_force geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: 'base_link'}, wrench: {force: {x: 5.0}}}"
```

**必ず機体を拘束した状態で** PWM 範囲・軸符号を確認してから水中運用すること。

### 7.4 やってはいけないこと

- `mavlink_driver` と別の MAVLink RC override クライアント（例: 旧 `blue.py`）の**同時実行**
- DVL の `dvl75_setup.launch.py` と `dvl75_driver.launch.py` の**同時起動**
- ARM 前・heartbeat なしでの推力指令（ドライバーが中立を送るが、運用上も避ける）

---

## 8. 関連パッケージ（上層との接続）

| パッケージ | 使うトピック | BlueROV 時の注意 |
|---|---|---|
| `localization` | `/driver/dvl`, `/driver/imu`, `/driver/depth` | そのまま使える |
| `dashboard` | 上記 + 各種 driver | そのまま使える |
| `joy2wrench`, `wrench_planner`, `p_pid_controller` | `robot_force` | remap 先を mavlink_driver に変更 + heartbeat 追加 |
| `driver_launcher` | `blue_rov_driver.launch.py` | BlueROV 用の一括起動入口 |

---

## 9. 詳細ドキュメント（リポジトリ内）

| パス | 内容 |
|---|---|
| `mavlink_driver/README.md` | MAVLink 変換表・パラメータ一覧・確認コマンド |
| `dvl75_driver/README.md` | DVL ドライバーの ROS インターフェース |
| `dvl75_driver/README_DVL75.md` | DVL-75 デバイス仕様・UDP・初期設定 |
| `blue_rov_msgs/README.md` | メッセージ設計の境界 |
| `driver_launcher/README.md` | 機体別 launch 入口 |

---

## 10. クイックリファレンス

```bash
# ビルド
cd /kyubic_ros/kyubic_ws && source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-up-to mavlink_driver dvl75_driver --symlink-install
source install/setup.bash

# 起動
ros2 launch driver_launcher blue_rov_driver.launch.py

# 状態確認
ros2 topic echo /driver/blue_rov/mavlink_driver/vehicle_state

# DISARM（緊急時含む）
ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool "{data: false}"
```
