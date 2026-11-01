cmake_minimum_required(VERSION 3.8)
project(mvp_basics)

# C++ 标准:rclcpp 要求 C++17
set(CMAKE_CXX_STANDARD 17)

# 常见警告打开,代码写得规范些(学习阶段有帮助)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找齐依赖包(对应 package.xml 里声明的依赖)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 关键一步:让 colcon 从 msg/ 和 srv/ 生成 C++ 代码(msg 与 srv 同源机制)。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hello.msg"
  "srv/SetHello.srv"
)

# 两个可执行文件:talker(发布)、listener(订阅)
# 消息生成库不是普通库,要用两步拿到底层的 typesupport 目标再链接:
#   1) rosidl_get_typesupport_target 查出该消息类型对应的链接目标
#   2) target_link_libraries 链接它,可执行文件才能 include 生成的消息/服务头文件
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp)
target_link_libraries(talker "${cpp_typesupport_target}")

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp)
target_link_libraries(listener "${cpp_typesupport_target}")

# 装到哪:可执行文件进 lib/包名,launch 文件进 share/包名(ros2 run/launch 按此查找)
install(TARGETS talker listener
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
