cmake_minimum_required(VERSION 3.8)
project(mybot_navigation)

# 本包不再编译 Go2 的 rotation_shim C++ 插件：
# 差速小车改用 nav2 原生 RegulatedPurePursuit（自带原地转向），免编译。
find_package(ament_cmake REQUIRED)

# 安装 launch / config 到 share
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Python 脚本作为可执行入口安装到 lib/<pkg>/，供 ros2 run 与 launch 使用
install(
  PROGRAMS
    scripts/nav_command.py
    scripts/wait_for_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
