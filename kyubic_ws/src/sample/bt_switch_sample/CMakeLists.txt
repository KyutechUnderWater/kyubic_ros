cmake_minimum_required(VERSION 3.8)
project(bt_switch_example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_cmake と必要な依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 実行ファイルの定義
add_executable(switch_test src/main.cpp)

# 依存関係のリンク
# behaviortree_cpp パッケージは BehaviorTreeV4::behaviortree_cpp ターゲットを提供する
# (ROS 2のバージョンやBTのバージョンによってターゲット名が異なる場合があるが、v4系は通常これ)
ament_target_dependencies(switch_test
  behaviortree_cpp
  ament_index_cpp
)

# インストール設定
install(TARGETS
  switch_test
  DESTINATION lib/${PROJECT_NAME}
)

# XMLファイルを share ディレクトリにインストール
install(DIRECTORY
  bt_xml
  DESTINATION share/${PROJECT_NAME}
)

# ament のフック
ament_package()
