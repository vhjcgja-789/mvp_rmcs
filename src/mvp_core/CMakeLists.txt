# RMCS 对照：组件仍由插件加载，但注册为原生 Node，同时生成独立入口。
cmake_minimum_required(VERSION 3.16)
project(mvp_core)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(mock_motor_component SHARED src/hardware/mock_motor.cpp)
rclcpp_components_register_node(mock_motor_component
  PLUGIN "mvp_core::hardware::MockMotor"
  EXECUTABLE mock_motor
)

ament_auto_add_library(pid_controller_component SHARED src/controller/pid/pid_controller.cpp)
target_include_directories(pid_controller_component PRIVATE src)
rclcpp_components_register_node(pid_controller_component
  PLUGIN "mvp_core::controller::pid::PidController"
  EXECUTABLE pid_controller
)

# 阶段 7 框架:启动接线校验器(规格期望话题 vs 真实 ROS 图,fail-fast)。
# 契约基类 component_base.hpp 为 header-only,随 include 目录分发。
ament_auto_add_executable(wiring_checker src/framework/wiring_checker.cpp)

# 阶段 7 模板组件:演示 claim/produce 契约与自动接线,不含控制。
ament_auto_add_library(template_source_component SHARED src/template/template_source.cpp)
target_include_directories(template_source_component PRIVATE src)
rclcpp_components_register_node(template_source_component
  PLUGIN "mvp_core::template_::TemplateSource"
  EXECUTABLE template_source
)

ament_auto_add_library(template_processor_component SHARED src/template/template_processor.cpp)
target_include_directories(template_processor_component PRIVATE src)
rclcpp_components_register_node(template_processor_component
  PLUGIN "mvp_core::template_::TemplateProcessor"
  EXECUTABLE template_processor
)

ament_auto_package()
