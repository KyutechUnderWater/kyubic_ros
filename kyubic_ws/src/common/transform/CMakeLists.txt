cmake_minimum_required(VERSION 3.8)
project(transform)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Define INTERFACE library
add_library(${PROJECT_NAME} INTERFACE)

# このライブラリが公開するインクルードディレクトリ
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
)

# 2. 上記で準備したエクスポート情報をファイルとしてインストール先に書き出す
#    これにより、find_package(transform) した側がこのターゲットをインポートできるようになる
install(EXPORT ${PROJECT_NAME}_targets
  FILE        ${PROJECT_NAME}_targets.cmake # ターゲット情報ファイル
  NAMESPACE   ${PROJECT_NAME}::             # インポート時の名前空間
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# find_package で読み込まれる設定ファイル (...Config.cmake) を生成
configure_package_config_file(
  "${PROJECT_NAME}Config.cmake.in" # <-- このテンプレートファイルを元に
  "${PROJECT_NAME}Config.cmake"    # <-- この設定ファイルを生成する
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

# 生成した設定ファイルをインストール
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(transform_test test/test_transform.cpp)
  target_link_libraries(transform_test
    ${PROJECT_NAME}
  )
endif()

ament_package()
