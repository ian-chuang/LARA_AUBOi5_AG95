cmake_minimum_required(VERSION 3.0.2)
project(lara_moveit_servo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_servo
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lara_moveit_servo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  pose_tracking_server src/pose_tracking_server.cpp
)
add_dependencies(pose_tracking_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_tracking_server ${catkin_LIBRARIES} )