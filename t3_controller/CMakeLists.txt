cmake_minimum_required(VERSION 3.0.2)
project(t3_contorller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  t3_msgs
  xycar_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
