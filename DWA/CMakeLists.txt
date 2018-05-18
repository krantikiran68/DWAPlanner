cmake_minimum_required(VERSION 2.8.3)
project(DWA)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_func
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf tf2
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  dwa_func src/dwa_func.cpp
 )

add_executable(dwa_node src/dwa_node.cpp)

target_link_libraries(dwa_func ${catkin_LIBRARIES})
target_link_libraries(dwa_node dwa_func ${catkin_LIBRARIES})

