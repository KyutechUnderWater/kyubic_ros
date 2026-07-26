# driver_launcher

機体ごとに必要なドライバーをまとめて起動するROS 2 launchパッケージ。

## Kyubic

Logic Distro RP2040、Sensors ESP32、IMUドライバーを起動する。DVLとActuatorは、従来どおり`kyubic_post.launch.py`から起動する。

```bash
ros2 launch driver_launcher kyubic_driver.launch.py
```

## BlueROV

MAVLinkドライバーとDVL-75ドライバーを通常運用設定で起動する。DVL-75の初期設定には、このlaunchではなく`dvl75_setup.launch.py`を使用する。

```bash
ros2 launch driver_launcher blue_rov_driver.launch.py
```

どちらのlaunchも`log_level`引数を各ドライバーへ渡す。既定値は`info`。
