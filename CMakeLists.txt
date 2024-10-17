cmake_minimum_required(VERSION 3.8)
project(sd_external_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(action_files
  "action/TaskExecution.action"
)
set(msg_files
  "msg/BoundingBox2DArray.msg"
  "msg/Gripper.msg"
  "msg/Weight.msg"
)
set(srv_files
  "srv/GripperControl.srv"
  "srv/Pause.srv"
  "srv/Reset.srv"
  "srv/Resume.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs vision_msgs
)

ament_package()
