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

add_message_files(
  FILES
  DistanceAndOrientation.msg
  RelativeCoordinates.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # include other package messages if available
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_odometry
 CATKIN_DEPENDS message_runtime nav_msgs roscpp rospy tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(odom_listener src/odom_listener.cpp)
target_link_libraries(odom_listener ${catkin_LIBRARIES})
add_executable(odom_listener_with_new_custom_msg_and_topic src/odom_listener_with_new_custom_msg_and_topic.cpp)
target_link_libraries(odom_listener_with_new_custom_msg_and_topic ${catkin_LIBRARIES})
add_executable(robot_coordinate_and_distance_orientation src/robot_coordinate_and_distance_orientation.cpp)
target_link_libraries(robot_coordinate_and_distance_orientation ${catkin_LIBRARIES})
add_executable(saving_robot_info src/saving_robot_info.cpp)
target_link_libraries(saving_robot_info ${catkin_LIBRARIES})
add_executable(robot_move_square src/robot_move_square.cpp)
target_link_libraries(robot_move_square ${catkin_LIBRARIES})
