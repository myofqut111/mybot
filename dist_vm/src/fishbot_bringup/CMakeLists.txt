cmake_minimum_required(VERSION 3.8)
project(fishbot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 说明：本包【不再编译 odom2tf】。
# odom -> base_link 的 TF 统一交给 robot_localization 的 EKF 发布
# （建图 mapping.launch / 导航 navigation.launch 内启动）。
# src/odom2tf.cpp 仅保留作参考，切勿再运行，否则与 EKF 重复发布同一条 TF。

# 安装 launch 与脚本
install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
