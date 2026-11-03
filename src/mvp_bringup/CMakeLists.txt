cmake_minimum_required(VERSION 3.28)
project(mvp_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
