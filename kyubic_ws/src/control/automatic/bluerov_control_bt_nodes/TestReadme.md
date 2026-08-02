# BlueROV2Auto デバッグ手順書(初学者向け)

対象: `behavior_tree/bt_xml/iwakuni2026.xml` の `BlueROV2Auto`(BlueROVの自動ミッション本体)。
実行パッケージは `bluerov_control_bt_nodes`(BT葉ノード)+ `behavior_tree`(BT実行エンジン)+
`wrench_planner`/`zero_order_hold`(PID・保持)+ `emergency`(緊急浮上)。

この文書は2つの環境で「何を」「なぜ」確認するかをまとめたチェックリストです。

- **環境A: ROS2ソフトウェアのみ**(実機・水無し。PCだけで完結)
- **環境B: 実環境**(直径6m・水深1mのテスト用プール)

対応する詳細設計・既知の制約は以下も参照してください(この文書はその要約+手順です):
- [README.md](README.md) §7(既知の制約・TODO)、§10(iwakuni2026コース、現行構成)
- [/blueNote.md](../../../../../blueNote.md)(mavlink_driver/dvl75_driverの安全設計・ARM手順)
- [/CLAUDE.md](../../../../../CLAUDE.md)(クロスビークルのトピック規約)

## 0. なぜ2つの環境に分けるか

|  | 環境A: ROS2ソフトウェアのみ | 環境B: 実環境(直径6m・水深1m) |
|---|---|---|
| 確認できること | XML構造、remapの繋がり、タイムアウト設計、Condition/Actionの分岐ロジック | DVLの水中ロック、ARM/DISARMの安全機構、緊急浮上時に実際に浮くか、リードスイッチ、ハイドロフォン実データ |
| 確認できないこと | ハードウェアが絡む物理的な挙動全般 | **大会本番スケール(数十m規模)のミッションをフル通しすること**(このプールでは物理的に狭すぎる。§A-9/B-11参照) |
| 何が壊れても | PCの再起動だけで済む | 機体がプールの壁・底に当たる、水没する等、物理的な被害が出うる |

→ **ロジックのバグは環境Aで先に全部潰す**。環境Bは「ソフトウェアだけでは検証できないこと」に絞る、が鉄則です。

## 起動コマンド(共通の前提)

環境A・Bどちらでも、起動順は同じです(3が現行のBT構成)。

```bash
# 1. ドライバ(mavlink_driver + dvl75_driver + camera_driver + reed_switch_driver)
ros2 launch driver_launcher blue_rov_driver.launch.py

# 2. センサ融合(位置推定) -> /localization/odom
ros2 launch localization localization_components.launch.py

# 3. BT + wrench_planner(PID) + zero_order_hold + emergency + buoy_detector
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py
```

このほかに、`bluerov_bt.launch.py` は音響(ハイドロフォン)を**起動しません**。ハイドロフォン方向を
使う手順(A-6以降、B-9)では別ターミナルで以下も起動します。

```bash
ros2 launch hydrophone_driver hydrophone_driver.launch.py
```

**重要**: `旧bluerov_control`(`ros2 launch bluerov_control bluerov_control.launch.py`)とは
**絶対に同時起動しない**こと。両方が`robot_force`を`mavlink_driver`へ送ろうとして競合します。

---

## 環境A: ROS2ソフトウェアのみでの確認

### A-1. ビルドが通ること

**なぜ**: コンパイルエラーはハードウェアの話以前の問題。まずここで潰す。

```bash
cd kyubic_ws
colcon build --packages-up-to bluerov_control_bt_nodes hydrophone_driver --symlink-install
source install/setup.bash
```

### A-2. 各launchが単独でエラー無く起動すること

**なぜ**: launchファイルの構成ミス(パッケージ名の綴り間違い、ノード名重複など)は、実際に
BTをtickさせる前に見つけたほうが原因の切り分けが速い。

```bash
ros2 launch driver_launcher blue_rov_driver.launch.py
ros2 launch localization localization_components.launch.py
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py
```

それぞれ別ターミナルで起動し、エラーログが出ないこと・`ros2 node list` に想定ノード
(`btExecutorNode`, `emergencyNode`, `planner/wrench_planner/*` など)が並ぶことを確認する。

### A-3. remap先トピックの疎通確認

**なぜ**: `bluerov_bt.launch.py` はBT葉ノードの相対トピック名(`odom`, `pinger_direction`,
`buoy_detection` など)を実トピック名へ大量にremapしています。1箇所でも間違えると
「エラーは出ないが、そのノードにだけデータが届かない」という気づきにくい不具合になります。

```bash
ros2 topic list
ros2 topic hz /driver/imu
ros2 topic hz /driver/depth
ros2 topic hz /driver/dvl
ros2 topic echo /localization/odom
```

### A-4. BT構造の目視確認(Groot2、静的チェックのみ)

**なぜ**: `iwakuni2026.xml` は1ファイルに `MainTree`/`BlueROV2Auto`/`Configure`/`Emergency` の
4つの木が入っており、`Fallback`/`Sequence`/`Timeout`/`RetryUntilSuccessful` の入れ子が深いです。
tickさせて動かす前に、Groot2でXMLを開いて接続・分岐の意図とコード上のコメントが一致しているか
目で確認しておくと、実行時ログだけでは追いにくい構造ミスに早く気付けます。

このリポジトリの `bt_executor` には現状Groot2のライブモニタリング(publisher)は組み込まれて
**いない**ため、Groot2は「XMLファイルを開いて眺めるだけ」の使い方になります。実行中の進行状況は
次のA-5で説明する `/talker` ログで代用してください。

### A-5. 実行中の進行状況をログで追う

**なぜ**: ミッションは`Talker`ノードで各フェーズの開始/終了を`/talker`にpublishしています。
Groot2のライブモニタが無い分、ここを見るのが「今どのフェーズか」を知る一番手軽な方法です。

```bash
ros2 topic echo /talker
```

### A-6. `main_tree` 引数での部分起動

**なぜ**: `MainTree` はバッテリーチェック→`Configure`→センサー正常性リトライ(10回)→
リードスイッチ待ち、を経てからでないと自動ミッションに入りません。机上で「ミッション本体だけ」
「緊急浮上だけ」を素早く繰り返し試すには、`main_tree`引数でこれらの前段をスキップします。
なお `iwakuni2026.xml` には(旧Phase 2と違って)フェーズ単位の個別SubTree IDは無く、
選べるのは `MainTree` / `BlueROV2Auto` / `Configure` / `Emergency` の4つだけです。

```bash
# ミッション本体だけを起動(バッテリー/リードスイッチ待ちをスキップ)
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=BlueROV2Auto

# 緊急浮上だけを起動
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=Emergency
```

### A-7. ダミーデータ注入でCondition/Actionの分岐を確認

**なぜ**: ハイドロフォン(`hydrophone_driver`)は実機のシリアル接続が無いと数値が出ず、
画像処理(`perception/buoy_detector`)はYOLOモデル・カメラ校正が未整備(README §7)なため実データが
来ません。ロジック(閾値判定・CSV書き込み・分岐)だけを先に確認するには、別ターミナルから
ダミー値をpublishします(機体を拘束・水無しの状態で問題なし)。

```bash
# ハイドロフォン方向のダミー値(CheckPingerPitch / WriteHydrophoneWaypointCSVが購読)
ros2 topic pub /pinger_direction planner_msgs/msg/PingerDirection \
  "{yaw: 0.0, pitch: 35.0, score: 0.01}" --rate 5

# ブイ検出のダミー値(CheckBuoyInFrame / CheckBuoyPosition / WriteVisionWaypointCSVが購読)
ros2 topic pub /buoy/relative_position buoy_interfaces/msg/BuoyRelativePosition \
  "{detected: true, yaw_valid: true, position_valid: true, confidence: 0.9}" --rate 5
```

`pitch`の値を変えて`CheckPingerPitch`の閾値(追跡ループ50°、解放判定70°)をまたぐか、
`confidence`を0.5未満にして`CheckBuoyInFrame`がFAILUREに戻るか、なども確認しておくと良いです。

### A-8. Lifecycle遷移コマンドの確認

**なぜ**: 起動シーケンス(`Configure`サブツリー、`Emergency`サブツリー)は`configure`→`activate`
という明示的なLifecycle遷移に依存しています。BT側の`LifecycleManager`ノードが内部で呼んでいるのと
同じ操作をCLIから直接呼べるので、BTを介さずに遷移そのものが通るかを先に確認できます。

```bash
ros2 lifecycle list /emergencyNode
ros2 lifecycle set /emergencyNode configure
ros2 lifecycle set /emergencyNode activate
```

### A-9. 静的waypoint CSVの座標がプール(直径6m・水深1m)に収まるかを机上で確認する(重要)

**なぜ**: `WaypointAction`が読む静的CSV(`path_planner/assets/iwakuni/`配下)は大会会場向けの
座標を書き込む前提のファイルです。値をそのまま実プールで使うと、壁や底に衝突する座標を
指定してしまう恐れがあります。実プールに入れる前に、必ず中身を目で見て確認してください。

```bash
cat kyubic_ws/src/control/automatic/planning/path_planner/assets/iwakuni/transit_shallow.csv
cat kyubic_ws/src/control/automatic/planning/path_planner/assets/iwakuni/descend_deep.csv
cat kyubic_ws/src/control/automatic/planning/path_planner/assets/iwakuni/return_final.csv
```

現時点(2026-08-02)の内容を確認した結果:

- `transit_shallow.csv` / `descend_deep.csv`: 目標座標は `(x, y, z) = (0, 0, 0.3m)` の1点のみ。
  実質「その場で深さ0.3mへ」という無害な内容なので、直径6m・水深1mのプールでもそのまま
  使って問題ありません。
- `return_final.csv`: `(x, y) = (11, 0)` m 相当の点を含みます。直径6mのプール(中心から半径3m)
  に対して座標が大きすぎるため、**このステップをそのまま実プールで実行しないこと**。
  実プール用に一時的に小さい座標へ書き換えるか、このステップの手前で意図的にBTを止めて
  次のテスト項目に進んでください。

このファイル自体は今も編集中(git上でも変更あり)なので、**テストのたびに毎回catして
中身を確認する**ことを習慣にしてください。

---

## 環境B: 実環境(直径6m・水深1mプール)での確認

環境Aで論理的な不具合を洗い出したうえで、**ハードウェアが絡む部分だけ**を実機で確認します。

### B-1. DVL-75初回セットアップ(まだ済んでいない場合のみ・1回だけ)

**なぜ**: `SEND-DVEXT ON`/`SEND-DVPDL ON`はDVL側の不揮発設定です。一度設定すれば以降は不要なので、
毎回のプールテストの前ではなく最初の1回だけ行います。

```bash
ros2 launch dvl75_driver dvl75_setup.launch.py
# $DVPDL が来ることを確認したら Ctrl-C
# 通常のdvl75_driver.launch.pyとは絶対に同時起動しないこと
```

### B-2. ドライバ接続確認(水に入れる前)

**なぜ**: 水に入れてから接続不良に気付くと、原因が「ソフト」なのか「配線・電源」なのか
切り分けに時間がかかります。入水前に必ず確認します。

```bash
ros2 launch driver_launcher blue_rov_driver.launch.py
ros2 topic echo /driver/blue_rov/mavlink_driver/vehicle_state
ros2 topic hz /driver/dvl
```

### B-3. ARM/DISARM手順の確認(推力を出す前に必須)

**なぜ**: `mavlink_driver`は明示的に`set_armed`サービスでARMしない限り中立PWMしか送らない
安全設計です(自動ARMはしない)。全員が即座にDISARMできる状態を作ってから水に入れます。

```bash
# ARM
ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool "{data: true}"

# 緊急停止(DISARM)
ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool "{data: false}"
```

### B-4. 手動操縦での推力方向・軸符号の確認(自動ミッションの前に必ず)

**なぜ**: `robot_force`の軸対応・符号(invert)は設定ファイル依存で、実際に機体がどちらへ動くかは
実機でしか確認できません。**注意**: 既存の`joy2wrench`パッケージのlaunchファイルはKyubic用の
トピック(`/driver/actuator_rp2040_driver/robot_force`)に固定remapされており、現状BlueROV用には
そのまま使えません(blueNote.md §7.1参照)。代わりに、`robot_force`と`heartbeat`を直接
`ros2 topic pub`する最小テストで確認します(blueNote.md §7.3)。

```bash
# heartbeat(別端末で流し続ける。無いとmavlink_driverが中立PWMに戻す)
ros2 topic pub -r 10 /driver/blue_rov/mavlink_driver/heartbeat std_msgs/msg/Bool "{data: true}"

# ARM
ros2 service call /driver/blue_rov/mavlink_driver/set_armed std_srvs/srv/SetBool "{data: true}"

# 前進指令の例(force.x = forward)。まずは機体を拘束した状態で試すこと
ros2 topic pub -r 10 /driver/blue_rov/mavlink_driver/robot_force geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: 'base_link'}, wrench: {force: {x: 5.0}}}"
```

### B-5. DVLの水中ロック確認

**なぜ**: DVLは水面(水底)からの反射を使って速度を測るため、ビームロック品質・confidenceは
水中でしか確認できません。環境Aでは絶対に検証できない項目です。

```bash
ros2 topic echo /driver/blue_rov/dvl75_driver/dvl75
ros2 topic echo /localization/odom
```

### B-6. リードスイッチによるミッション開始トリガー確認

**なぜ**: `MainTree`は`wait_for_trigger=true`で`reed_switch_driver`の物理トリガーを待つ設計
(ジョイスティック操作なしで自動的に`BlueROV2Auto`へ入る)なので、実機のスイッチでしか
確認できません。

```bash
ros2 topic echo /driver/blue_rov/mission_start_trigger
```

### B-7. 緊急浮上(Emergency)の実機動作確認

**なぜ**: `emergencyNode`をactivateしてZOHを止め、浮上指令を送る「操作」自体はA-6・A-8で
確認済みですが、実際に機体が浮くかどうか(浮力・スラスタ出力が十分か)はソフトウェアだけでは
分かりません。

```bash
ros2 launch bluerov_control_bt_nodes bluerov_bt.launch.py main_tree:=Emergency
```

実際に機体が浮上を始めることを目視確認する。想定外の挙動があればすぐDISARM(B-3)。

### B-8. センサー健全性チェックの既知の抜け穴(誤診断を防ぐために必読)

**現状の仕様(バグではなく未実装)**:
- `bluerov_bt.launch.py`では`leak`トピックのremapがコメントアウトされたままで、
  BlueROV用のリーク検知ドライバ自体もまだ存在しません(リークセンサーは今のところKyubic専用の
  `sensors_esp32_driver`にしかありません)。
- さらに`CheckSensorsStatus`の判定は`imu`/`depth`/`dvl`/`leak`の**OR**(1つでも健全なら
  SUCCESS)で実装されているため、leakが未配線でも他の3つが健全である限りSUCCESSを返し続けます。

**なぜ確認が必要か**: プールテストでリークを模擬的に発生させても、この状態では
機体は緊急浮上に入りません。これは「見つけたバグ」ではなく既知の未実装状態なので、
テスト結果を「緊急浮上ロジックが壊れている」と誤診断しないよう、テスト前に必ずこの制約を
共有しておいてください。

### B-9. ハイドロフォン追跡フェーズ(`hydrophone_driver`)

**なぜ**: 6chハイドロフォンアレイの実データはシリアル接続の実ハードウェアが無いと
生成できず、環境Aのダミー注入(A-7)では信号処理そのものは検証できません。

```bash
ros2 launch hydrophone_driver hydrophone_driver.launch.py serial_port:=/dev/ttyACM0
ros2 topic hz /pinger_direction
ros2 topic echo /pinger_direction
```

**注意**: `hydrophone_driver`はまだ作業中の新規パッケージです。`bluerov_control_bt_nodes/README.md`
にはこの音響入力が「`sbl_controller_node`という外部・未実装のノード」と書かれていますが、
`hydrophone_driver`はその代替として追加されたものです(READMEの記述はまだ更新されていません)。

### B-10. 画像処理(`buoy_detector`)の実機確認

**なぜ**: YOLOモデル・水中カメラ校正が未整備(README §7)なため、実際に水中でブイを検出
できるかどうかは実機でしか確認できません。

```bash
ros2 topic echo /buoy/relative_position
```

検出精度が不十分でこの先のテストが進められない場合は、A-7のダミー注入に戻して
BT側のロジック(センタリング・突撃・解放判定)だけを切り分けて確認してください。

### B-11. 静的waypointのスケール再確認(実行前に必ず・A-9の繰り返し)

A-9で確認した通り、`return_final.csv`はプールの外(半径3m超)を指す座標を含みます。
**実プールでミッションをフル通し実行する前に、必ずA-9のcatコマンドで中身を再確認**し、
プールに収まらないステップの手前でテストを打ち切るか、座標を一時的に書き換えてください。

### B-12. PIDゲイン・roll軸への注意(水深1mプールならではの制約)

**なぜ**: `config/p_pid_controller_gain_bluerov.yaml`は実機未チューニングの保守的な初期値で、
特にroll軸は旧`bluerov_control`が一度も制御したことがない軸です(README §7)。水深1mのプールは
競技プールに比べて姿勢が乱れた際の余裕(浮上までの距離)がずっと小さいため、はじめは
出力を絞った状態で少しずつ上げていくこと。

### B-13. 7分タイムアウト・フォールバックの確認

**なぜ**: `MissionCore`が7分(420000ms)以内に完了しない場合、`Fallback`が自動的に
`(11, 0, 9)`相当の座標への直行に切り替わります。この座標もプールの外を指すため、
タイムアウトのフォールバックが暴走に見えて慌てないよう、この段階に達する前に
テストを打ち切るか、`main_tree`(A-6)で狙ったフェーズだけを個別に実行してください。

---

## まとめ: 実プールに入れる前の最終チェックリスト

- [ ] 環境Aの A-1〜A-9 をすべて確認済み
- [ ] B-1(DVL初回セットアップ)は完了済み、または不要と確認済み
- [ ] B-2(接続確認)・B-3(ARM/DISARM手順)を全員が即座に打てる
- [ ] A-9 / B-11 の静的CSVを直近で`cat`して座標を再確認した
- [ ] B-8(リーク検知が現状効かないこと)をテスト立会者全員が理解している
- [ ] B-12(PIDゲイン、特にroll軸が未検証であること)を理解している
