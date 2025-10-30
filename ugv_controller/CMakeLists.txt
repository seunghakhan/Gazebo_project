cmake_minimum_required(VERSION 3.8)
project(ugv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(path_follower_node src/path_follower_node.cpp)
ament_target_dependencies(path_follower_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav_msgs
  tf2 
  tf2_ros              
  tf2_geometry_msgs 
  visualization_msgs
)

add_executable(manual_path_logger_node src/manual_path_logger_node.cpp)
ament_target_dependencies(manual_path_logger_node
  rclcpp
  tf2_ros
  geometry_msgs
)

add_executable(new_path_follower_node src/new_path_follower_node.cpp)
ament_target_dependencies(new_path_follower_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs    
  geometry_msgs
  nav_msgs
  visualization_msgs
)

install(TARGETS 
  path_follower_node
  manual_path_logger_node
  new_path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
