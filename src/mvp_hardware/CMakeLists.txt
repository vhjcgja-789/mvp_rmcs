cmake_minimum_required(VERSION 3.28)  # librmcs v3.1.0 的最低要求
project(mvp_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)  # librmcs 头文件用到 C++23
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# librmcs:板子的 USB 收发 SDK。
# 来源:RMCS/rmcs_ws/src/rmcs_core/CMakeLists.txt 第 16–24 行,原样照抄(同 URL、同 SHA256)。
# 它内部用 pkg-config 查找 libusb-1.0(依赖见 package.xml)。
include(FetchContent)
set(BUILD_STATIC_LIBRMCS ON CACHE BOOL "Build static librmcs SDK" FORCE)
FetchContent_Declare(
  librmcs
  URL https://github.com/Alliance-Algorithm/librmcs/releases/download/v3.1.0/librmcs-sdk-src-3.1.0.zip
  URL_HASH SHA256=07107e251745ddb23f7b3e39edec5d6910be1a197025d167ec9849c5c80dd954
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(librmcs)

# 硬件组件插件:每块板一个 SystemInterface
add_library(${PROJECT_NAME} SHARED src/c_board_system.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME}
  librmcs-sdk
  hardware_interface::hardware_interface
  pluginlib::pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface mvp_hardware.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

# 阶段 0 验证工具:构造一块 CBoard,确认容器内能连上板子
add_executable(board_probe tools/board_probe.cpp)
target_link_libraries(board_probe librmcs-sdk)
install(TARGETS board_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
