# RMCS 对照：bringup 只安装启动描述和配置，组件实现仍在 core 包。
cmake_minimum_required(VERSION 3.16)
project(mvp_bringup)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_package(INSTALL_TO_SHARE launch config)
