# educational_pid

深度と方位のPID制御を、ROS 2のTopicとServiceを使って学ぶための教材パッケージです。

既定の`educational_pid.launch.py`はPC専用の模擬ロボットへ接続します。実機のActuator Driverへは接続しません。実機用は`educational_pid_robot.launch.py`ですが、3日目には使用しないでください。

実機用launchは`joy_common`を起動し、`educational_pid`がジョイスティックからX・Y手動指令を作ります。既存の`manual.launch.py`または`joy2wrench.launch.py`を同時に起動しないでください。同時起動すると、Actuator Driverの`robot_force`に複数Publisherが接続します。

実機用launchだけではセンサ、Localization、Actuator Driverは起動しません。KYUBICの通常のBringup手順でこれらを起動し、Topic名、センサStatus、`robot_force`のPublisher数を確認してから使用します。実機用ゲインと出力上限は水中・係留状態で一軸ずつ再確認してください。

## ファイル

- `educational_pid/pid_core.py`：完成版PID計算
- `educational_pid/pid_node.py`：完成版Topic・Service登録
- `educational_pid/controller_node.py`：出力制限、センサ監視、Disable時ゼロ出力
- `educational_pid/mock_plant_node.py`：PC用の簡易深度・方位モデル
- `student_templates/`：受講者へ配布する虫食いファイル
- `docs/DAY3_PID_ROS2_GUIDE_JA.md`：3日目手順書
- `docs/DAY3_START_HERE_JA.md`：3日目の最初に読む短い導入・実行手順
- `docs/EDUCATIONAL_PID_STRUCTURE_GUIDE_JA.md`：各ファイルの役割・入出力・ファイル間構造の詳しい説明
- `test/test_pid_core.py`：PID計算の自動テスト

## PC用起動

```bash
ros2 launch educational_pid educational_pid.launch.py
```

PIDは起動直後には無効です。別端末から明示的に有効化します。

```bash
ros2 service call /educational_pid/enable std_srvs/srv/SetBool "{data: true}"
```

停止時は次を先に実行します。

```bash
ros2 service call /educational_pid/enable std_srvs/srv/SetBool "{data: false}"
```

PC用Launchでは、模擬ロボットが深度・Yawを統合した
`localization_msgs/msg/Odometry` を `/localization/odom` へ20 HzでPublishします。
`rt_pose_plotter` では `pose.position.z_depth`、`pose.orientation.z` と、
`/rt_pose_plotter/targets` にPublishされる目標値を表示できます。

PIDの内部状態は、次のデバッグTopicで確認できます。

```text
/educational_pid/debug/error_depth
/educational_pid/debug/error_yaw
/educational_pid/debug/integral_depth
/educational_pid/debug/integral_yaw
```

目標値は`target_depth`と`target_yaw` Parameterとして保持し、教材用の独自Service`/educational_pid/set_targets`から2値をまとめて変更します。

```bash
ros2 service call /educational_pid/set_targets \
  p_pid_controller_msgs/srv/SetTargets \
  "{target_depth: 1.0, target_yaw: 45.0}"
```

積分項と微分用の履歴は`/educational_pid/reset_pid`でリセットします。
実機Localizationと同じTopic名を使用するため、PC模擬実験中は実機Localizationを同時起動しないでください。
