# DVL-75デバイス情報

[ROS 2利用方法](README.md)を先に参照。

## 実装時の参照先

| 参照先 | 実装での用途 |
| --- | --- |
| [Cerulean Sonar DVL-75 documentation](https://docs.ceruleansonar.com/c/dvl-75) | 製品の用途・仕様の確認 |
| [Communicating with the DVL](https://docs.ceruleansonar.com/c/dvl-75/communicating-with-the-dvl) | Ethernet/TTL serialの通信方式、送受信の基本仕様 |
| [Ethernet Interface](https://docs.ceruleansonar.com/c/dvl-75/communicating-with-the-dvl/the-ethernet-interface) | DHCP、フォールバックIP、UDP、ブロードキャスト受信ポートの確認 |
| [$DVEXT format](https://docs.ceruleansonar.com/c/dvl-75/communicating-with-the-dvl/outgoing-message-formats-dvl-to-host/usddvext-dvl-extended-data) | `$DVEXT`のフィールド順、底面ロック、ビーム値、高度のパース |
| [$DVPDL format](https://docs.ceruleansonar.com/c/dvl-75/communicating-with-the-dvl/outgoing-message-formats-dvl-to-host/usddvpdl-dvl-position-and-angle-deltas-message) | `$DVPDL`の位置差分、角度差分、時刻、信頼度のパース |
| [DVL commands](https://docs.ceruleansonar.com/c/dvl-75/communicating-with-the-dvl/commands-accepted-by-the-dvl) | `SEND-DVEXT`、`SEND-DVPDL`、`HOST-ADDRESS`などの送信コマンド |
| [Coordinate Systems](https://docs.ceruleansonar.com/c/dvl-75/appendix-coordinate-systems) | DVL取付姿勢と座標系の確認 |
| [Kyubic Pathfinder driver](../../kyubic/dvl_driver) | Pathfinderドライバーの共通`driver_msgs/msg/DVL`利用、状態判定、launch構成の参照 |
| [driver_msgs/msg/DVL.msg](../../driver_msgs/msg/DVL.msg) | 共通Publishメッセージの正本 |
| [blue_rov_msgs/msg/DVL75.msg](../blue_rov_msgs/msg/DVL75.msg) | DVL-75固有Publishメッセージの正本 |

公式ドキュメントとDVLファームウェアの組合せにより出力形式が異なる可能性があるため、ファームウェア更新後は実際のUDPパケットを取得し、パーサー試験と`DVL75.msg`への対応を再確認すること。

## 通信・取得データ

DVL-75のUDP ASCII/NMEA文を受信。実装が処理する文は`$DVEXT`と`$DVPDL`。DVEXTは最新値として保持し、DVPDL受信時にだけ両者を統合した`DVL75`メッセージを1回Publishする。

### UDPブロードキャスト

DVL-75の工場出荷時設定は、UDPメッセージを`255.255.255.255:27000`へブロードキャスト送信する構成。本ドライバーは`bind_address: 0.0.0.0`および`listen_port: 27000`へbindして受信するため、同一ネットワーク上であればROSホストのIPをDVLへ設定しなくても計測データを受信可能。

受信パケットの送信元IPはDVL自身のIPであり、`dvl_address`と一致するパケットだけを受け入れる。例えば`192.168.2.111:50000 > 255.255.255.255:27000`の場合、`dvl_address`は`192.168.2.111`、`dvl_command_port`は`50000`。

初期設定専用の`dvl75_setup.launch.py`を実行し、かつ`device_host_address`を設定すると、`HOST-ADDRESS <IP>:<port>`をDVLへ送信する。これはDVLの送信先をユニキャストへ変更する永続設定であり、ブロードキャスト受信から変更する場合は、ROSホストのIP・ポートを正しく設定すること。

| 取得項目 | 内容 | ROS 2へのPublish |
| --- | --- | --- |
| `$DVEXT`の底面ロック | `T`/`F` | `DVL75.bottom_lock` |
| `$DVEXT`のビームロック | 4ビームの`T`/`F` | `DVL75.beam_lock` |
| `$DVEXT`のビーム速度・距離 | ビーム軸の速度[m/s]・距離[m] | `DVL75.beam_velocity`、`DVL75.beam_range` |
| `$DVEXT`の高度 | センサー軸方向の距離[m] | `DVL75.altitude`、有効時は`DVL.altitude` |
| `$DVPDL`の位置差分 | DVL車体座標系の差分[m] | `position_axis_sign`を反映して`DVL75.position_delta`へPublish。同じ値から`DVL.velocity`を算出 |
| `$DVPDL`の角度差分 | 差分[rad] | `angle_axis_sign`を反映して`DVL75.angle_delta`へPublish |
| `$DVPDL`の時刻・信頼度 | µs・0–100 | `DVL75.device_time_us`、`DVL75.delta_time_us`、`DVL75.confidence` |

`$DVEXT`はチェックサム直前の末尾カンマを含む形式を許容。`$`で始まらない起動・情報テキストは無視。

各ROSメッセージの`header.stamp`は、ドライバーがUDPパケットを受信した直後のROS時刻。DVL内部の計測時刻ではないため、DVL時刻・計測間隔の確認には`DVL75.device_time_us`・`DVL75.delta_time_us`を使用。

### `$DVPDL`の出力周期と速度更新

[Cerulean Sonarの`$DVPDL`仕様](https://docs.ceruleansonar.com/c/dvl-75/communicating-with-the-dvl/outgoing-message-formats-dvl-to-host/usddvpdl-dvl-position-and-angle-deltas-message)では、出力周期は固定ではなく5〜20 Hzの可変レート。共通トピック`DVL.velocity`は`$DVPDL`の受信時だけにPublishするため、速度の更新周期もこの出力周期に従う。ドライバーには周期を固定するパラメータはない。

`DVL75.delta_time_us`はDVL内部での前回`$DVPDL`からの時間差。実測周期と周波数は次で確認可能。

```text
周期 [s] = delta_time_us / 1,000,000
周波数 [Hz] = 1,000,000 / delta_time_us
```

例えば`delta_time_us: 50000`は20 Hz、`delta_time_us: 200000`は5 Hz。5 Hz時は速度値が最大約200 ms間隔で更新されるため、これは操作に対する見かけの遅れとなり得る。

## 初期化・永続設定

通常の`dvl75_driver.launch.py`は永続設定を変更しない。DVLを初めて接続するときだけ、次を実行する。

```bash
ros2 launch dvl75_driver dvl75_setup.launch.py
```

初期設定launchは`apply_device_config=true`を上書きし、以下をUDPでDVLへ送信する。

1. `device_host_address`が非空の場合: `HOST-ADDRESS <値>`
2. `SEND-DVEXT ON`
3. `SEND-DVPDL ON`
4. `startup_commands`の各行

`$DVEXT`と`$DVPDL`は本ドライバーに必須の出力。初期設定launchは両方を必ずONにしてDVLへ永続設定する。`$DVPDL`受信を確認したら`Ctrl-C`で停止し、以後は通常launchを使用する。通常launchは設定送信を省略するだけで、DVLがすでに送信している`$DVEXT`・`$DVPDL`の受信、速度計算、トピックPublishは停止しない。初期設定launchと通常launchは同時に起動しない。

`command`サービスおよび`startup_commands`でも、`SEND-DVEXT OFF`と`SEND-DVPDL OFF`は拒否する。過去の設定や別ツールによって両出力がすでにONなら、`apply_device_config: false`でも`$DVPDL`を受信して`DVL.velocity`を計算する。

`SEND-DVPDL ON`送信後に`$DVPDL`が未受信なら、速度情報が得られない構成異常としてERRORログを出力。

## PAUSE / RESUME (電源投入時の自動Ping対策)

DVL-75は電源投入と同時にPingを開始する。`resume_on_startup: true`（既定）の場合、ノード起動時に一度`RESUME`をDVLへ送信し、DVLが電源投入直後の状態のままでも本ドライバー起動と同時にPingを再開させる。`pause_on_shutdown: true`（既定）の場合、ノード終了時（`Ctrl-C`・プロセス終了）に一度`PAUSE`を送信し、本ドライバーが動いていない間はDVLを止めておく。

どちらも永続設定ではなく一時的な状態変更のコマンドなので、`apply_device_config`の値に関係なく動作する。DVLが未接続・未応答の場合は送信失敗をWARNログに残すのみで、ノードの起動・終了処理は妨げない。

## 品質判定

- 4本ロック: `NORMAL`
- 3本ロックかつ他条件成立: `WARNING`
- 底面ロックなし、ロック本数不足、信頼度不足、`DVEXT`欠落、時間差不正: `ERROR`
- チェックサム・フィールド形式が不正なDVEXT/DVPDL、DVPDL受信タイムアウト: 共通`DVL`を`ERROR`としてPublish

ERRORでは共通`DVL`の`velocity_valid=false`。DVL-75専用`DVL75`は品質判定にかかわらず、受信した生の計測値を保持。

## デバイス固有の確認

```bash
tcpdump -ni <interface> udp port 27000
```

受信パケットの送信元IPを`dvl_address`へ設定。送信元ポートは`dvl_command_port`の確認材料になるが、実装の設定値は明示的に確認すること。
