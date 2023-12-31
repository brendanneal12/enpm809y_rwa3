cmake_minimum_required(VERSION 3.8)
project(group12)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

# include directories
include_directories(include/group12)

# executables
add_executable(robot_controller src/robot_controller.cpp src/utils.cpp)
ament_target_dependencies(robot_controller rclcpp geometry_msgs nav_msgs mage_msgs tf2_ros tf2 tf2_geometry_msgs ros2_aruco_interfaces)
install(TARGETS robot_controller DESTINATION lib/${PROJECT_NAME})

# install directories
install(DIRECTORY include config launch DESTINATION share/${PROJECT_NAME})


ament_package()
