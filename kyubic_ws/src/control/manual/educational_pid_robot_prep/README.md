# educational_pid_robot_prep

インターン4日目用の、実機接続準備と安全確認を行うROS 2パッケージです。3日目の`educational_pid`を変更せず、その出力とActuator Driverの間にSafety Gateを追加します。

## 最初に読む資料

1. `docs/DAY4_START_HERE_JA.md`
2. `docs/DAY4_ROBOT_PREP_GUIDE_JA.md`
3. TAのみ：`docs/DAY4_TA_GUIDE_JA.md`

## 絶対に守ること

- `day4_student_dry_run.launch.py`の出力は、実機Actuator Topicへremapしない。
- `manual.launch.py`、`joy2wrench.launch.py`と`day4_robot.launch.py`を同時に起動しない。
- スラスタを空中で回さない。水中・係留・退避手段あり・TA立会いでのみ試験する。
- 緊急停止は`/educational_pid_day4/emergency_stop`を使用する。
- このGateは補助的なソフトウェア保護であり、物理的な非常停止の代わりではない。

現行の`actuator_rp2040_driver`は、Wrench callback内でHeartbeat timeoutを確認します。Wrench Publisherのプロセスが突然停止した場合、それだけで直ちにゼロ指令を送る独立タイマではありません。したがって、水中試験には物理遮断・マイコン側Watchdog・回収手段が必要です。

## Launchの使い分け

|Launch|入力|出力|用途|
|---|---|---|---|
|`day4_student_dry_run.launch.py`|実機センサ|`/educational_pid_day4/student_preview_robot_force`|受講者のコード演習。実機出力禁止|
|`day4_dry_run.launch.py`|実機センサ|`/educational_pid_day4/preview_robot_force`|完成版Safety Gateの事前確認|
|`day4_robot.launch.py`|実機センサ|`/driver/actuator_rp2040_driver/robot_force`|TA立会いの水中試験のみ|

`day4_robot.launch.py`は既定で深度・方位の両方を遮断します。試験する軸だけLaunch引数で明示的に有効化します。
