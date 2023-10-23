cmake_minimum_required(VERSION 3.8)
project(gps_odom)

find_package (ament_cmake_auto REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geodesy REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    geodesy
)

ament_auto_find_build_dependencies ()

ament_auto_add_library ( gps_odom SHARED 
    src/gps_odom.cpp 
    include/gps_odom/gps_odom.hpp)

rclcpp_components_register_node(gps_odom
  PLUGIN "lazy::GpsOdomNode"
  EXECUTABLE gps_odom_node
)

ament_target_dependencies(gps_odom ${dependencies})
ament_auto_package ()