cmake_minimum_required(VERSION 3.8)
project(pose_tf_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# 실행파일 추가
add_executable(pose_tf_broadcaster src/pose_tf_broadcaster.cpp)

# 의존성 연결
ament_target_dependencies(pose_tf_broadcaster
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_msgs
)

# 실행파일 설치
install(TARGETS
  pose_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
