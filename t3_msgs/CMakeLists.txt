cmake_minimum_required(VERSION 3.0.2)
project(t3_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BoundingBox.msg
  object_data.msg
  lidar_array.msg
  lidar_point.msg

)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime
)
