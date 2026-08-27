# インターン生用・虫食い課題

このフォルダには、完成版を見ずに小さな ROS 2 ノードを作るための課題ファイルがあります。`*_exercise.py` は**そのままでは実行できません**。`TODO` を埋めてから、下の表で指定した「配置先」にコピーしてビルド・実行してください。

`student_templates` だけでは実行できません。必ず同梱の `ros2_ws` と一緒に使います。

## 最初に 1 回だけ行う準備

ZIP を展開したフォルダで、次を実行します。`<展開した教材の場所>` は実際のフォルダ名に置き換えてください。

```bash
cd <展開した教材の場所>/ROS2_Jazzy_beginner_material
mkdir -p ~/ros2_internship_ws
cp -r ros2_ws/. ~/ros2_internship_ws/
cp -r student_templates ~/ros2_internship_ws/
```

以後、編集する虫食いファイルは `~/ros2_internship_ws/student_templates/` にあり、プログラムとして実行するファイルは `~/ros2_internship_ws/src/internship_ros2_basics/internship_ros2_basics/` にあります。

## 毎回共通の手順

1. 表の「編集する配布ファイル」を開き、`TODO` を埋めて保存する。
2. 表の「配置先」へコピーする。これは自分のワークスペース内の完成版を、作業中のファイルで置き換える操作です。元の教材 ZIP は変更されません。
3. ビルドしてから、表の「実行コマンド」を新しいターミナルで実行する。

例：トピック課題 1 を配置する場合

```bash
cp ~/ros2_internship_ws/student_templates/string_publisher_exercise.py \
  ~/ros2_internship_ws/src/internship_ros2_basics/internship_ros2_basics/string_publisher.py

cd ~/ros2_internship_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select internship_ros2_basics
source install/setup.bash
```

同じパッケージを再ビルドするときも、上の `source` と `colcon build` を行ってください。新しいターミナルでは `source /opt/ros/jazzy/setup.bash` と `source ~/ros2_internship_ws/install/setup.bash` が必要です。

## 課題と、編集・実行するファイルの対応表

配置先はすべて、次のフォルダの中です。

```text
~/ros2_internship_ws/src/internship_ros2_basics/internship_ros2_basics/
```

|通信|編集する配布ファイル|配置先のファイル名|起動前に必要なもの|実行コマンド・確認方法|
|---|---|---|---|---|
|練習問題1・Topic：文字列を定期送信|`student_templates/string_publisher_exercise.py`|`string_publisher.py`|なし|端末 A：`ros2 run internship_ros2_basics string_subscriber`<br>端末 B：`ros2 run internship_ros2_basics string_publisher`|
|練習問題2・Topic：カメを円運動|`student_templates/turtle_circle_publisher_exercise.py`|`turtle_circle_publisher.py`|端末 A で `ros2 run turtlesim turtlesim_node`|端末 B：`ros2 run internship_ros2_basics turtle_circle_publisher`<br>必要なら端末 C：`ros2 run internship_ros2_basics turtle_pose_subscriber`|
|練習問題3・Service：足し算サーバ|`student_templates/add_two_ints_server_exercise.py`|`add_two_ints_server.py`|なし|端末 A：`ros2 run internship_ros2_basics add_two_ints_server`<br>端末 B：`ros2 run internship_ros2_basics add_two_ints_client 7 11`|
|練習問題4・Service：カメを追加|`student_templates/spawn_turtle_client_exercise.py`|`spawn_turtle_client.py`|端末 A で `ros2 run turtlesim turtlesim_node`|端末 B：`ros2 run internship_ros2_basics spawn_turtle_client`<br>課題ファイルの値どおり、`student_turtle` が追加されれば成功|
|練習問題5・Action：カメを90度回転|`student_templates/turtle_rotate_action_client_exercise.py`|`turtle_rotate_action_client.py`|端末 A で `ros2 run turtlesim turtlesim_node`|端末 B：`ros2 run internship_ros2_basics turtle_rotate_action_client`|
|練習問題6・Action：フィボナッチ列|`student_templates/fibonacci_action_server_exercise.py`|`fibonacci_action_server.py`|下記の「Action 用ビルド」を先に実行|端末 A：`ros2 run internship_ros2_basics fibonacci_action_server`<br>端末 B：`ros2 run internship_ros2_basics fibonacci_action_client 8`|

## Action 用ビルド

フィボナッチのアクションは、`internship_interfaces` という独自の Action 定義も使います。最初に 1 回、またはそのワークスペースを消した後には、次のように 2 パッケージをまとめてビルドしてください。

```bash
cd ~/ros2_internship_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select internship_interfaces internship_ros2_basics
source install/setup.bash
```

虫食い版のフィボナッチ・サーバは、まず「フィードバックと結果を返す」ことに集中した簡易版です。キャンセル処理を試すのは、完成版 `fibonacci_action_server.py` に戻してからにしてください。

## 困ったときの確認順

1. 編集したのは `student_templates` 内の正しいファイルか。
2. そのファイルを、表に書かれた名前でパッケージ内へコピーしたか。
3. コピー後に `colcon build` を実行したか。
4. 実行するターミナルで `source ~/ros2_internship_ws/install/setup.bash` を実行したか。
5. turtlesim を使う課題では、別ターミナルで `ros2 run turtlesim turtlesim_node` を起動しているか。
