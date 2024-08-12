cmake_minimum_required(VERSION 3.5)
project(project_14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(camera_setup src/camera_setup.cpp)
ament_target_dependencies(camera_setup rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(line_follower src/line_follower.cpp)
ament_target_dependencies(line_follower rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

install(TARGETS
  camera_setup
  line_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
