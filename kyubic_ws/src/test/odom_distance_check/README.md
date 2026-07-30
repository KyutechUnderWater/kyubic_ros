# odom_distance_check

既知の距離だけ実際に機体(またはセンサー)を手で移動させ、その前後の`/localization/odom`の位置差分を
記録する、**読み取り専用**の検証ツール。`dvl_odometry_component`がDVL速度を積分して計算する
`pos_x`/`pos_y`が、実際の物理的な移動距離とどれだけ一致するかを、後で人間が手動で突き合わせるための
記録を残す。

**このノードは`WrenchPlan`等を一切publishしない**(購読のみ)。安全機構(ランプ/クランプ等)は
不要な設計。`sample/blue_control`(`blue_control_listener`)と同じ「購読専用」の設計方針を踏襲している。
このツールにpublisherやservice clientを追加しないこと(安全レビュー無しに追加しないという方針は
`blue_control`と同じ)。

対象読者: ROS2に触れ始めたばかりの開発者。

## 1. 使い方

**`ros2 launch`ではなく`ros2 run`で起動すること**。このツールは`input()`によるキーボード対話で
マークを記録するが、`ros2 launch`が起動する子プロセスは標準入力がターミナルに接続されない
(`/dev/null`相当になる)ことがあり、その場合`input()`が即座にEOFとなり何もマークできずに
セッションが終了してしまう。`launch/odom_distance_check.launch.py`は用意してあるが、
**パラメータのデフォルト値を確認する目的以外では使わないこと**。

```bash
# 1. センサ融合(位置・姿勢推定)を先に起動しておく
ros2 launch localization localization_components.launch.py

# 2. このツールを ros2 run で起動する(標準入力がそのままターミナルに繋がる)
ros2 run odom_distance_check odom_distance_check --ros-args \
  -p odom_topic:=/localization/odom \
  -p output_dir:=~/odom_distance_check_logs
```

起動後、ターミナルで以下を操作する:

- **空行(Enterキーのみ)**、または任意のメモ文字列を入力してEnter: **マーク**
  - 1回目のマークは「原点」として記録するだけで、まだCSVには何も書かれない
  - 2回目以降のマークで、直前のマークとの間の区間が1件確定し、CSVへ即座に追記される
  - メモ文字列を入力した場合、その文字列がその区間の`note`列に記録される(巻尺の実測値などを
    その場でメモしておく用途。空でもよい)
- **`done`/`q`/`quit`/`exit`**(または`Ctrl+C`/`Ctrl+D`): 記録を終了する。全区間のサマリーが
  ターミナルに表示され、CSVファイルのパスが案内される

### 操作の流れ(例)

```
> [Enter]                  # 原点でマーク(mark 0)
(機体を巻尺で測った距離だけ手で移動させる)
> 3.0m migi                # 区間1を確定。noteに"3.0m migi"を記録
(さらに移動させる)
> [Enter]                  # 区間2を確定。noteは空
> done                     # 終了、サマリー表示
```

## 2. 記録される項目(CSV)

1ファイル1セッション: `output_dir/odom_distance_check_<起動日時>.csv`(既定`output_dir`は
`~/odom_distance_check_logs`)。1行 = 区間1件(マークとマークの間)。

| 列 | 内容 |
|---|---|
| `segment_index` | 区間番号(1始まり) |
| `start_stamp_sec`/`start_stamp_nanosec`、`end_stamp_sec`/`end_stamp_nanosec` | 区間の始点・終点のodomメッセージの`header.stamp`(ROS時刻。rosbagと突き合わせる際はこの時刻を使う) |
| `start_x`/`start_y`/`start_z_depth`、`end_x`/`end_y`/`end_z_depth` | 始点・終点の`pose.position` |
| `dx`/`dy`/`distance_m` | 区間の水平変位量と直線距離(`distance_m = hypot(dx, dy)`) |
| `dz_depth` | 区間の深度変化量 |
| `elapsed_time_s` | 区間の経過時間(odomの`header.stamp`基準) |
| `worst_depth_status`/`worst_imu_status`/`worst_dvl_status` | 区間中に観測した各センサーstatusの最悪値(`NORMAL`/`WARNING`/`ERROR`。1つでもERRORがあればERROR) |
| `cumulative_distance_from_origin_m` | 最初のマーク(mark 0、原点)からその区間の終点までの直線距離。複数区間を連続して記録した際に、積算誤差がどれだけ蓄積しているかを見るための値 |
| `note` | マーク時に入力した任意のメモ文字列(巻尺実測値など。空でもよい) |

## 3. ⚠ ERRORが混ざった区間の結果は参考値として扱うこと

Part A(`bluerov_control_bt_nodes/README.md`§9、DVL/depth/IMUがERRORの間にゼロ値が
`/localization/odom`へ伝播していたバグの修正)により、いずれかのセンサーがERRORの間、
`odom`の該当フィールドは**ゼロへは飛ばず直前の有効値を保持する**ようになっている。つまり
`worst_dvl_status`等がERRORの区間でも、`odom`の値自体はいきなり不自然な値にはならない。

しかし、保持された値は「その瞬間に実測された値」ではなく「最後に分かっている推定値」でしかない。
ERRORが混ざった区間でDVLが実際には動きを検出できていない(値が更新されていない)可能性があるため、
**この区間の`distance_m`等は参考値として扱い、DVL積分精度の評価には使わないこと**。
`worst_depth_status`/`worst_imu_status`/`worst_dvl_status`のいずれかが`NORMAL`でない区間は、
端末出力でも`⚠`付きで警告される。

## 4. 正解距離をツールに入力させない方針について

正解距離(巻尺等の実測値)は、このツール自体には入力させない**記録専念型**の第1版とした
(依頼者と合意済み)。理由:

- プールサイドでの入力の手間を最小化し、記録漏れのリスクを下げる
- 誤差(%)の自動計算は便利だが、今回の主目的は「まず記録を残すこと」
- `note`列に実測値を手でメモしておけば、後で表計算ソフト等で`distance_m`と突き合わせて
  誤差を計算できる

将来的に余裕があれば、マーク時に`--expected_distance`のような形で正解値を入力させ、
自動で誤差(%)を計算・表示する機能を追加する余地は残してある。

## 5. 既知の制約

- `ros2 launch`経由では対話入力ができない可能性がある(§1参照)。`ros2 run`を使うこと
- 1セッション=1プロセスの実行時間内のみ記録する。プロセスを再起動すると新しいCSVファイルに
  なる(セッションをまたいだ連続性は無い)
- 正解距離との自動比較は行わない(§4参照)
