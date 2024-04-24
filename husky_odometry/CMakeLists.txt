cmake_minimum_required(VERSION 3.0.2)
project(husky_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  tf
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_odometry
#  CATKIN_DEPENDS nav_msgs roscpp rospy tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  DistanceAndOrientation.msg
)

generate_message(
  DEPENDENCIES
  std_msgs  # include other package messages if available
)

add_executable(odom_listener src/odom_listener.cpp)
target_link_libraries(odom_listener ${catkin_LIBRARIES})

