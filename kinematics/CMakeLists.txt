cmake_minimum_required(VERSION 3.8)

project(kinematics)

add_definitions(-std=c++17 -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_library(kinematics src/pose.cpp)
ament_export_targets(kinematics HAS_LIBRARY_TARGET)

ament_target_dependencies(kinematics
    rclcpp
)

install(
    DIRECTORY include/kinematics
    DESTINATION include
)

install(
   TARGETS kinematics
   EXPORT kinematics
   LIBRARY DESTINATION lib
   ARCHIVE DESTINATION lib
   RUNTIME DESTINATION lib
   INCLUDES DESTINATION include
)

ament_package()
