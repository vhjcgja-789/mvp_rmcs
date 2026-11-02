# RMCS 对照：像 rmcs_msgs 一样单独生成接口，原生组件通过消息交换数据。
cmake_minimum_required(VERSION 3.16)
project(mvp_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float64Stamped.msg"
  DEPENDENCIES std_msgs
)

ament_auto_package()
