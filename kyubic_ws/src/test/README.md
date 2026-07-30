# `src/test/` — 実機データ検証ツール置き場

このディレクトリは、本番のBT/制御コード(`behavior_tree/`、`control/`等)とは別に、
**実機データの妥当性を検証するためのツールを置く場所**。「何があって、どう使って、
何が確認できれば合格なのか」をここ1つで把握できることを目指す。個別ツールの詳しい使い方は
各ツールのREADMEを参照し、ここでは概要と全体の位置づけだけを扱う。

## 1. なぜ本番コードと分離しているか

本番の`behavior_tree`/BT XML/制御ノードには、検証目的のためだけの変更を一切加えない、という
設計方針を採っている(`bluerov_control_bt_nodes`のPhase 1/2移行でも一貫している方針)。
検証ツールは本番コードを購読するだけの独立したノードとして追加し、本番の挙動に影響を与えない
ようにする。これにより、検証ツールのバグや未完成な実装が、実機を制御する本番コードの安全性に
影響することを防ぐ。

## 2. ツール一覧

| ツール | 検証内容 | 実機(水中)要否 |
|---|---|---|
| [`odom_distance_check`](odom_distance_check/README.md) | オペレーターが既知の距離だけ機体(またはセンサー)を手で動かし、その前後の`/localization/odom`の位置差分を記録する。`dvl_odometry_component`のDVL速度積分結果が、実際の物理的な移動距離とどれだけ一致するかを検証する | 必須(読み取り専用。`WrenchPlan`等は一切publishしない) |

今のところ`src/test/`配下のツールはこの1つのみ。新しいツールを追加する運用ルールは§5を参照。

## 3. 実験の進め方(概要)

詳細な手順は各ツールのREADMEを参照。ここでは最初の一歩だけを示す。

### `odom_distance_check`

1. `localization_components.launch.py`を先に起動し、`/localization/odom`が流れていることを確認する
2. `ros2 run odom_distance_check odom_distance_check`で起動する(**`ros2 launch`では対話入力できない
   場合があるため使わないこと**。詳細は[odom_distance_check/README.md](odom_distance_check/README.md)§1)
3. 機体(またはセンサー)を巻尺等で測った既知の距離だけ手で動かし、移動前後でEnterキーを押してマークする
4. 3〜5m程度の区間を5〜10区間ほど、往復や複数方向を含めて記録することを推奨
5. `done`で終了し、出力されたCSV(既定`~/odom_distance_check_logs/`)の`distance_m`と、手元でメモした
   実測値を突き合わせて誤差を確認する

## 4. 何が得られれば良い結果か

- **ステータス異常が混ざった区間は参考値として除外する**: `odom_distance_check`が記録する
  `worst_depth_status`/`worst_imu_status`/`worst_dvl_status`のいずれかが`NORMAL`でない区間は、
  DVL精度評価には使わない。Part A(`bluerov_control_bt_nodes/README.md`§9、DVL/depth/IMUが
  ERRORの間にゼロ値が`odom`へ伝播していたバグの修正)により`odom`はいきなり不自然な値には
  ならないが、保持されているのは「最後に分かっている推定値」であり「その瞬間の実測」では
  ないため
- **`distance_m`の許容誤差(何%以内なら合格か)は、本ドキュメント作成時点では確定していない**。
  参考値として、DVL-75公式ドキュメント(Cerulean Sonar、`dvl75_driver/README_DVL75.md`が参照
  している`docs.ceruleansonar.com`)には以下の記載がある:

  > Dead Reckoning Circular Error Probable (CEP) = median expected error: **5% distance traveled**

  ただしこれは**中央値(median)であり保証値ではなく**、動作条件(ボトムトラック前提かどうか等)の
  明記もない公式スペックである。したがって「誤差5%以内なら合格」という単純な基準として
  そのまま使うことはできない。**実際の合否基準は、`odom_distance_check`でプールテストにより
  実測したデータをもとに、事後的にチームで決定する。** 上記5%はあくまで比較のための参考値
  (「実測誤差がこれより大きければ、公称の中央値すら下回っている」という下限の目安)として扱うこと

## 5. 新しいツールを追加するときの運用ルール

`src/test/`配下に新しいツール(ダミーodom配信ツール、ダミー検出データ配信ツール等)を追加したら、
このREADMEの§2の表にも1行追記すること。ドキュメントが実態と乖離しないようにするための運用ルール。
