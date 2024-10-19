cmake_minimum_required(VERSION 3.8)
project(moving_ur5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ur_robot_driver REQUIRED)


add_executable(move_group_interface_ur_manipulator src/move_group_demo.cpp)
ament_target_dependencies(move_group_interface_ur_manipulator
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  ur_robot_driver
)

add_executable(move_group_reciever src/move_group_reciever.cpp)
ament_target_dependencies(move_group_reciever
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  ur_robot_driver
)

add_executable(move_group_publisher src/move_group_publisher.cpp)
ament_target_dependencies(move_group_publisher
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  ur_robot_driver
)

install(TARGETS move_group_publisher move_group_reciever move_group_interface_ur_manipulator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
