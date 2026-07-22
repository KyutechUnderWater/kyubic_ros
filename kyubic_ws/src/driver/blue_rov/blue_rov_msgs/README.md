# blue_rov_msgs

BlueROV固有のROS 2メッセージ定義パッケージ。車両間で共通化するメッセージは`driver_msgs`など既存の共通パッケージに置き、BlueROVまたは接続デバイス固有の値だけを本パッケージに置く構成。

## ビルド

```bash
cd <workspace>
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select blue_rov_msgs --symlink-install
source install/setup.bash
```

## メッセージ

| 型 | 用途 |
| --- | --- |
| `blue_rov_msgs/msg/DVL75` | Cerulean Sonar DVL-75の`$DVEXT`および`$DVPDL`固有データ |

`DVL75`は、DVL-75ドライバーの`/driver/blue_rov/dvl75_driver/dvl75`から`$DVPDL`受信ごとに1回Publishされる。最新DVEXTを統合できた場合は`dvext_valid=true`、`dvpdl_valid`は常に`true`。

`header.stamp`はホストがUDPパケットを受信した直後のROS時刻。`device_time_us`と`delta_time_us`はDVL内部の相対計測時刻および計測間隔であり、通信遅延の解析と共通`DVL.velocity`の算出根拠を確認するため、DVL-75専用メッセージに残す。

## 設計上の境界

- `driver_msgs/msg/DVL`: PathfinderとDVL-75で共有する速度・高度・状態
- `blue_rov_msgs/msg/DVL75`: ビーム状態、DVL時刻、位置・角度差分、信頼度などDVL-75固有の情報

この分離により、DVL-75固有フィールドの追加が共通`DVL`メッセージの互換性へ影響しない構成。

## 確認

```bash
ros2 interface show blue_rov_msgs/msg/DVL75
```

メッセージ内容とDVL-75の通信仕様は、[DVL-75ドライバーのデバイス資料](../dvl75_driver/README_DVL75.md)を参照。
