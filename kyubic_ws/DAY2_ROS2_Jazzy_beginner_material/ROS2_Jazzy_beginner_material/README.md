# ROS 2 Jazzy 初学者向け教材

ROS 2を知らないインターン生が、PCだけで次の順に学ぶ教材です。

1. ROS 2を使う理由とROSグラフ
2. turtlesimとCLIによる観察
3. TopicのPublisher / Subscriber
4. ServiceのServer / Client
5. ActionのGoal / Feedback / Result / Cancel
6. 自作Pythonノードのビルドと実行
7. 配布Pythonファイルを編集する練習問題1〜6
8. Topic・Service・ActionをCLIで試す確認問題A〜C

## 教材ファイル

- `docs/ROS2_BEGINNER_GUIDE_JA.md`：講義と実習を一体化した主教材。各問題の直前に、編集・配置・ビルド・実行手順があります
- `docs/EXERCISE_WORKSHEET_JA.md`：インターン生が記入する課題
- `docs/TA_GUIDE_JA.md`：進行、答え、確認ポイント
- `ROS2_Jazzy_beginner_material.pptx`：参考用PowerPoint（Markdownで授業する場合は使用不要）
- `ros2_ws/`：動作する完成版ROS 2ワークスペース
- `student_templates/`：配布用の虫食いコード。どのファイルを編集・配置・実行するかは、`student_templates/README_STUDENT.md`に一覧があります

## 対象環境

- Ubuntu 24.04
- ROS 2 Jazzy
- Bash
- ROS 2、turtlesim、rqt、colconを使用できる環境

別OSの場合はsourceやパスの書式を読み替える必要があります。本教材のコマンドはUbuntu/Bashを基準にしています。

## 完成版を最初に動かす

ZIPを展開したディレクトリで次を実行します。

```bash
mkdir -p ~/ros2_internship_ws
cp -r ros2_ws/. ~/ros2_internship_ws/
cp -r student_templates ~/ros2_internship_ws/
cd ~/ros2_internship_ws

source /opt/ros/jazzy/setup.bash
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

最初の確認です。

```bash
ros2 interface show internship_interfaces/action/Fibonacci
ros2 pkg executables internship_ros2_basics
```

以降の実習は`docs/ROS2_BEGINNER_GUIDE_JA.md`の順に進めてください。
コード課題では、本文中の各「練習問題」「確認問題」の直下にある手順を上から実行してください。最初の対応表へ戻らなくても、各問題だけで編集対象、コピー先、ビルド方法、端末ごとの実行方法を確認できます。`student_templates/README_STUDENT.md`は全課題を一覧で確認したいときに使います。

## 参照元

ROS 2 Jazzy公式チュートリアルの次の章を基礎にしています。

- Configuring environment
- Using turtlesim, ros2, and rqt
- Understanding nodes / topics / services / actions
- Creating a workspace / package
- Writing a simple publisher and subscriber (Python)
- Writing a simple service and client (Python)
- Creating an action
- Writing an action server and client (Python)

公式入口：https://docs.ros.org/en/jazzy/Tutorials.html
