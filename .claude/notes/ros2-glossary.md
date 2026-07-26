## colcon
- 定義: ROS 2 ワークスペース内の複数パッケージをまとめてビルドするツール（make の上位にあたる「ワークスペース全体のビルド指揮者」）
- このコードでの実例: `docker/once_entrypoint.sh` の `colcon build --symlink-install --cmake-args -GNinja`、エイリアス `build`
- 関連: CMakeLists.txt / setup.py（各パッケージのビルド定義）、install/setup.bash（ビルド成果物の読み込み）
- 解説日: 2026-07-24

## build（エイリアス）
- 定義: `colcon build --symlink-install --cmake-args -GNinja` の短縮コマンド。ソース変更を実行可能なノード・launch に反映するために使う
- このコードでの実例: `docker/once_entrypoint.sh` で `.bash_aliases` に登録
- 関連: `ros2 launch`（ビルド後に起動する）、`install/` ディレクトリ（ビルド出力先）
- 解説日: 2026-07-24

## source install/setup.bash
- 定義: colcon が `install/` に置いたビルド成果物を、今のシェルが `ros2` から見つけられるように登録するコマンド（apt install とは別物）
- このコードでの実例: `docker/Dockerfile` が `.bashrc` に追記。コンテナ起動時に自動 source
- 関連: `/opt/ros/jazzy/setup.bash`（ROS本体）、`AMENT_PREFIX_PATH` / `PATH`
- 解説日: 2026-07-24

## CMakeLists.txt
- 定義: C++ パッケージ（ament_cmake）のビルド設計書。何をコンパイルし、どの ROS ライブラリにリンクし、どこに置くかを cmake に指示する
- このコードでの実例: `dvl75_driver/CMakeLists.txt` — `dvl75_parser` と `dvl75_driver` 実行ファイルを生成
- 関連: `package.xml`（依存の宣言）、`colcon build`
- 解説日: 2026-07-24

## setup.py
- 定義: Python パッケージ（ament_python）のビルド・インストール設計書。モジュール配置と `ros2 run` 用コマンド名を登録する
- このコードでの実例: `mavlink_driver/setup.py` — `mavlink_driver` コマンド → `mavlink_driver.py:main` を登録
- 関連: `package.xml`（`<build_type>ament_python</build_type>`）、`entry_points`
- 解説日: 2026-07-24
