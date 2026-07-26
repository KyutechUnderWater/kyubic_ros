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
