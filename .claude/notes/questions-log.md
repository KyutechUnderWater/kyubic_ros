## Q: kyubic_ws の実行方法
- 状況: リポジトリ全体 / kyubic_ws（ROS 2 Jazzy ワークスペース）
- 期待 vs 実際: 実行手順がわからない（初回セットアップ〜起動まで）
- 回答要約: Docker で `ros2_start` → コンテナ内で `build`（colcon）→ `byobu` または `ros2 launch kyubic_bringup kyubic.launch.py` 等で起動。初回は `install.sh` と submodule 取得が必要。
- 日付: 2026-07-24

## Q: build と colcon は何のため？
- 状況: コンテナ内の `build` エイリアス / `colcon build`
- 期待 vs 実際: 実行前に build が必要な理由と colcon の意味がわからない
- 回答要約: ソース（.cpp/.py）を `ros2 launch` で動かせる実行ファイル・メッセージ型に変換する工程。colcon はワークスペース全体のビルドツール。`build` はその短縮形。コードを変えたら再 build が必要。
- 日付: 2026-07-24

## Q: setup.bash・ビルドファイル・launch 順序の意味
- 状況: `install/setup.bash`、`CMakeLists.txt`、`mavlink_driver/setup.py`、byobu の3 launch
- 期待 vs 実際: 「install」が何か、ビルドファイルの役割、launch の起動順がわからない
- 回答要約: setup.bash は install/ 成果物をシェルに登録（apt とは別）。CMakeLists.txt/setup.py は言語別ビルド設計書。launch 順は論理依存（ドライバ→推定→行動ツリー）だが ROS は強制しない。
- 日付: 2026-07-24
## Q: blue_control_listener を launch したのに ros2 topic list に何も出ない、正常か？
- ユーザー: EaLux88
- 状況: kyubic_ws/src/sample/blue_control/src/blue_control_listener.cpp（Subscribe専用ノード）
- 期待 vs 実際: launch後にtopic listへ新しいトピックが増えると思っていたが、増えなかった
- 回答要約: 正常。Subscriberだけでは新規トピックは生まれない（Publisherが存在して初めてtopic listに載る）。貼られたlistに購読対象7トピックが全部既に存在していたので配線自体は正しい。受信確認は `ros2 node list` / `ros2 topic info <topic> -v` の Subscription count、または launch実行中ターミナルの RCLCPP_INFO_THROTTLE ログ出力で行う。
- 日付: 2026-07-26

## Q: blue_control_listenerでdepthしかログが出ない、これで合ってる？
- ユーザー: EaLux88
- 状況: blue_control_listener 実行中。ros2 node list / ros2 topic info /driver/imu -v の出力を確認
- 期待 vs 実際: imu/dvl/dvl75/vehicle_state/power_state/odom も出ると思っていたが depth: しか出ない
- 回答要約: 正常。現在起動しているのはBlueROV用(mavlink_driver/dvl75_driver)ではなくKyubic用sensors_esp32_driverで、その中のdepth_component_nodeだけが共通トピック/driver/depthにremapされているため。Publisher count: 0 (/driver/imu)がそれを裏付ける。driver_launcher blue_rov_driver.launch.py（+必要ならlocalization）を起動し直す必要がある。Kyubic用とBlueROV用ドライバは同時起動しないこと。
- 日付: 2026-07-26

## Q: odom_stale緊急浮上フェイルセーフの追加内容と目的が分からない
- ユーザー: EaLux88
- 状況: bluerov_control/bluerov_control/control.py, node.py (直前のセッションでmasudanote.md Day4に基づき追加した変更)
- 期待 vs 実際: 何を・何のためにやったのかが分からない、という漠然とした疑問
- 回答要約: odomが一定時間(odom_timeout_s、既定1.0s)途絶えたら、Stateに関わらず強制的にEMERGENCYへ遷移させるフェイルセーフを追加。理由は、古いodom値のままPID制御を続けると計算が狂って暴走しうるため(masudanote.md Day4の警告に基づく)。node.pyの_odom_callbackで受信時刻を記録→_tick_fnで経過時間を計算しodom_staleを算出→control.pyのis_emergency(position_z, odom_stale)に渡し、位置が浅すぎる場合と同じ既存のEMERGENCY経路(emergency_surfacing起動)に合流させる形で実装。
- 日付: 2026-07-26

## Q: リードスイッチ実行時に起動されるものと、Blue用BTでBlueROVを動かすために追加すべきコードは？
- ユーザー: pilot
- 状況: reed_switch_driver / bluerov_control_bt_nodes / bluerov_phase2.xml
- 期待 vs 実際: リードスイッチでミッション(BT)が動き出すと思っていたが、実際はUI表示とライト点滅のみ。Blue BTはこれから全部書く必要があると思っていたが、Phase 2は実装済み。
- 回答要約: リードスイッチ反応時は reed_switch_driver が (1) Pi LED点滅 (2) mavlink_driver/led 経由でBlueROVライト点滅 (3) /driver/blue_rov/mission_start_trigger を1回publish するだけ。BT/旧FSMはこのトピックを購読しておらず、ミッション開始は init_wait_sec(既定180s)の固定Delayで決まる。Blue BTは bluerov_control_bt_nodes + bluerov_phase2.xml + bluerov_bt.launch.py が既にあり、起動は driver→localization→bluerov_bt + 手動ARM。競技本番でリードスイッチ連動するなら WaitForMissionStart 等の新BTノード、bt_executor登録、launch remap、MainTree XML変更が必要。旧bluerov_controlとBTは同時起動不可(robot_force競合)。
- 日付: 2026-07-31

## Q: 視覚的に、現在リードスイッチが起動するとどうなるか？
- ユーザー: pilot
- 状況: reed_switch_driver / bluerov_dashboard / mavlink_driver(led)
- 期待 vs 実際: リードスイッチでミッションやスラスターが動き出すイメージがあるかもしれないが、実際は「点滅＋トピック1回＋ダッシュボード表示」だけ
- 回答要約: 磁石のON/OFFでGPIO検知→reed_switch_driver._on_triggerが同時に3動作(Pi LED 0.2s点滅、BlueROVライトPWM 1900→0.2s後1100、mavlink_driver経由)、mission_start_triggerを1回publish。購読者はbluerov_dashboardのみ(LED緑・経過秒数・イベントログ)。BT/制御/ARM/スラスターは一切動かない。
- 日付: 2026-07-31

## Q: 自己位置推定の原点の取り方 / IMU のみで DVL 利用可能水深まで移動する実装方針は？
- ユーザー: pilot
- 状況: localization/dvl_odometry_component, imu_transform_component / bluerov_phase2.xml / BlueROV Phase 2
- 期待 vs 実際: 地上起動→プール投下のため原点がズレる。DVL ロックできない浅い水深では IMU だけで自己位置推定・移動したい。その後 DVL ロック地点を改めて原点にしたい。
- 回答要約:
  ① 原点リセットは `/localization/reset` サービスを任意のタイミングで呼ぶだけ(dvl xy=0, IMU 姿勢=0, 深度=0 になる)。地上で localization 起動→投下直後にサービス呼び出し→その地点が原点。
  ② DVL ERROR 中は dvl_odometry が x/y を0保持するだけで、IMU加速度からの位置積算はコードに存在しない。「IMU だけで x/y を測る」機能は現状ない。
  ③ 推奨実装方針: 「GoToDepth で目標水深まで潜る(深度センサのみ)→DVL NORMAL になるまで待機→再び /localization/reset を呼んでそこを xy 原点にする→通常ウェイポイント移動」。DVL ロック後の 2 回目 reset を BT ノード化するには CaptureMissionOrigin + ResetLocalization を組み合わせた新 BT ノードを作るのが最小コスト。
  ④ IMU 姿勢はリセットで「その瞬間 = 0」になるが、水中で姿勢が乱れた後は offset が動いてしまう。地上キャリブレーション値を使い続けたい場合は imu_transform の reset() を水に入る前（地上でキャリブ後）に1回だけ呼び、以降は呼ばないようにする必要がある(現状は ResetLocalization が IMU も同時にリセットしてしまうため、IMU だけリセットしないよう localization/reset の動作を分離することも検討が必要)。
- 日付: 2026-07-31

## Q: LifecycleManager(DeactivateManualNode) は何をしているか？
- ユーザー: pilot
- 状況: behavior_tree/bt_xml/iwakuni2026.xml L6-9 / Auto ツリーの先頭ノード
- 期待 vs 実際: ノードの動作が不明
- 回答要約: `manualNode`（手動操縦 Wrench 担当）を deactivate 状態に遷移させる BT アクション。自動モード開始直後に手動制御を止めることで、manual/auto 両ノードが同時に robot_force を送る競合を防ぐ。timeout_sec="1" で 1秒以内に遷移完了しなければ BT 失敗扱い。
- 日付: 2026-07-31
