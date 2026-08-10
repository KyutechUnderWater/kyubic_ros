# pinger_search_control

BlueROV 向けのピンガー探査ミッション制御パッケージ。**DVL・localization を使わず**、
IMU の yaw (`/driver/imu` の `orient.z` [deg]) と深度計 (`/driver/depth`) のみを状態量として、
推力 (`robot_force`, `geometry_msgs/WrenchStamped`) を直接制御する単独 FSM ノード
`pinger_search_node` を提供する。

`bluerov_control` / `bluerov_control_bt_nodes` とは独立した実装。**どちらとも同時に起動しないこと**
(いずれも `mavlink_driver` の `robot_force` へ送信するため、制御が競合する)。

## ファイル構成

| ファイル | 役割 |
|---------|------|
| `pinger_search_node.py` | ROS I/O層: 購読・配信・set_armedサービス・PID・センサ鮮度管理 |
| `mission_fsm.py` | ミッションFSM本体。1ステート=1メソッドで、docstringに step番号/動作/遷移条件を記載。遷移図もファイル冒頭にある |
| `pid.py` | 深度・ヨー保持用の単段PID |
| `pinger_filter.py` | ピンガー外れ値フィルタ(score棄却+メディアン) |

## ミッションフロー

| # | State | 内容 |
|---|-------|------|
| 0 | `WAIT_START` | `/driver/blue_rov/mission_start_trigger`(リードスイッチ)が true になるのを待つ。**その瞬間の yaw を Yaw1 として記憶**する |
| 0' | `START_DELAY` | トリガ後 `start_delay_s` (30秒) 中立のまま待機。9分タイムアウトの起点はここ |
| 0'' | `ARMING` | `set_armed(true)` を呼ぶ(応答が無ければ3秒ごとにリトライ) |
| 1 | `INITIAL_DASH` | RF x = 30 N を 3 秒印加(yaw は Yaw1 を保持)して停止 |
| 2 | `DIVE` | 深度 4 m まで潜航し保持 |
| 3 | `PINGER_APPROACH` | 深度 4 m 保持のまま「**停止して観測**(外れ値除去平均で方位決定。既に正面なら即前進) → 旋回 → **停止して再観測・正面か確認**(ズレていれば旋回に戻る) → **前進**(ヨー制御なし・ヨーPIDのI項リセット) → 小休止」を繰返す。詳細は `_chase_wrench()`。ピンガー「上」判定が `pinger_above_hold_s` (10秒)連続したら通り過ぎ確定とみなし、そのまま浮上して終了 |
| 4 | `FINAL_DESCEND` | ピンガー伏角が 60° 以上(ほぼ真下)になったら、0.5 m ずつ深度目標を下げつつ、ピンガーの pitch が正面に来るまで z・yaw を制御。正面**ピッチ**条件が `pinger_front_confirm_s` (3秒) **連続で成立**したら次へ |
| 4' | `RISE_ALIGN` | ピンガーは目標物の**下側**に付いているため、その深度から `charge_rise_m` (0.2 m) **浮上**し、浮上先で step3 と同じ「停止観測→旋回→再確認」で**ヨーを合わせ直す**(前進はしない)。合ったら `enable_vision: true` → VISION_ALIGN / false → CHARGE。タイムアウト時は突進せず離脱 |
| 5 | `VISION_ALIGN` | (`enable_vision: true` 時) `/perception/image_processing_enable` (std_msgs/Bool) を true にし、**ライトを点灯**(`light_pwm`、mavlink_driver の `led` へ)。YOLO が**検出できている間**は水平正規化偏差 (`/buoy/relative_position`) で yaw 目標のみ微調整して画面中央(左右)へ合わせる — **深度は画処理では動かさず** step4' の深度を保持。**未検出の間**(`buoy_timeout_s` 途絶)は step3 と同じピンガーアプローチで真上に留まる。検出が戻れば中心合わせに復帰 |
| 6 | `CHARGE` | `charge_force_n` で計 `charge_duration_s` 秒前進(突進)。`enable_vision: true` なら中央合わせ後、**false なら step4' のヨー合わせ後に直接ここへ入る**(step5-6 の代替)。突進完了後は深度目標を `post_charge_lift_m` 浅くし、**`charge_force_n` で前進を続けながら、まず通常の深度PIDで浮上を試みる**(ブイに引っかかっていなければこれで普通に上がる)。`post_charge_lift_trial_s` 以内に到達できなければ「ブイに引っかかった」と判定し、深度PIDではなく固定の `post_charge_lift_force_n` (最大推力) に切り替えて `post_charge_lift_duration_s` 秒引っ張って外す。いずれの経路でも完了後は `RETREAT_GOTO` へ。「上」判定が `pinger_above_hold_s` (10秒)連続したら中断し、`RETREAT_GOTO` を飛ばして直接後進(`RETREAT_BACK`、ヨーもYaw1に戻す)→cruise深度→`SURFACE_CHECK` |
| 7 | `RETREAT_GOTO` → `RETREAT_BACK` | 画処理 bool を false。深度 4 m・Yaw1 に整定後、5 秒後進して停止 |
| 8 | `SURFACE_CHECK` | 深度 4 m 保持でピンガー pitch を再観測(最低 5 秒 + フィルタ窓分の新規サンプル)。ピンガーがロボットより **上** → 1 m へ浮上して終了。まだ **下** → step 3 へ戻る |
| 9 | (常時監視) | 開始から 9 分超過で `ABORT_ASCEND`: 強制的に 1 m へ浮上して終了 |
| - | `PAUSE` | 1 動作(ダッシュ・潜航・前進・突進・後進など)が終わるたびに挟まる小休止。`action_pause_s` (2秒) の間、前後推力を止めて深度・ヨーだけ保持し、機体を落ち着かせてから次のステートへ。step3/音響追尾の前進バースト間にも同じ休止が入る |
| - | `DISARMING` → `FINISHED` | 終了時に `set_armed(false)`。以後は中立 wrench を送信し続ける |

安全機構:

- **全条件待ちステートにタイムアウト**: `state_timeout_s` (120 s、`PINGER_APPROACH` のみ
  `approach_timeout_s` = 300 s) を超えたら次のステートへ強制遷移し、どこでもハングしない。
  浮上・DISARM系は先に進める方が安全側(必ず終了に到達)。ARM できないときは終了、
  DISARM 未確認のまま終わるときはエラーログを出す(手動 DISARM すること)。
  `FINAL_DESCEND` のタイムアウト時は正面未確認のまま突進はせず、そのまま離脱(step7)へ。
- imu / depth が `sensor_timeout_s` (1 s) 途絶えたら中立 wrench を出す(制御凍結)。
- `heartbeat` (std_msgs/Bool, true) をノード自身が 5 Hz で送信 — mavlink_driver の
  `require_control_heartbeat` フェイルセーフに対応。
- 深度目標は `max_depth_m` (8 m) を超えない。
- ARM はトリガ後に明示的にサービスで行い、終了・タイムアウト時は必ず DISARM する。

## ピンガー外れ値対策

`/pinger_direction` (planner_msgs/PingerDirection) は推定値がたまに飛ぶため、2 段でフィルタする:

1. `score` (最適化誤差、小さいほど良い) が `pinger_score_max` を超えるサンプルは棄却。
2. 直近 `pinger_filter_window` (既定 3) サンプルの **メディアン** を採用
   (yaw は円環メディアン)。単発スパイクは除去され、旋回による正しい変化には約 1 サンプル遅れで追従。
3. さらに step3 の旋回目標は、停止した状態で `pinger_avg_samples` (既定 2) 回観測し、
   メディアンから `pinger_avg_outlier_deg` (既定 20°) 超ズレたサンプルを捨てた
   **外れ値除去平均** で決める(観測は collect/recheck の停止中のみ行う)。
   ピンガーは約 1 Hz 更新なので観測回数を増やすほど停止時間が延びる — 単発の飛びは
   2 のメディアンで既に除去されるため 2〜3 回で十分。

## 角度・符号の規約

ピンガー(確認済み):

- `yaw` : ロボット正面 = 0°、右 = 正(+180°まで)、左 = 負(-180°まで) → `pinger_yaw_sign: 1.0`
- `pitch` : 上 = 正、下 = 負 → `pinger_depression_sign: -1.0`
  (内部では伏角 = `-pitch`、正 = 真下方向として扱う)

推力(実機で逆に動いたらパラメータで反転):

- `wrench.force.z` 正 = 潜航方向 (mavlink_driver の heave 軸。`axis.heave.invert` と整合させる)
- `wrench.torque.z` 正 = 右旋回 (`yaw_torque_sign` で反転可)

## 起動

```bash
ros2 launch driver_launcher blue_rov_driver.launch.py        # mavlink_driver ほか
# reed_switch_driver / ハイドロフォン(/pinger_direction) / buoy_detector も別途起動
ros2 launch pinger_search_control pinger_search.launch.py                # 本番(4m)
ros2 launch pinger_search_control pinger_search.launch.py \
    config_file:=pinger_search_pool_0_7m.param.yaml                      # 練習用0.7mプール
```

パラメータセットは `config/` に3種類:

- `pinger_search.param.yaml` — 本番想定 (保持深度4m、上限8m、0.5m刻み)
- `pinger_search_pool_0_7m.param.yaml` — 水深0.7mの練習プール用
  (保持0.4m / 上限0.6m / 0.1m刻み、推力・時間も弱め・短め)
- `pinger_search_pool_3m.param.yaml` — 水深3mの練習プール用
  (保持2.0m / 上限2.5m / 0.25m刻み。推力・PIDは0.7mプールの実機合わせ値を引き継ぎ)

ゲインは未チューニングの保守的な初期値なので、実機 (プール) でのチューニングが必須。
