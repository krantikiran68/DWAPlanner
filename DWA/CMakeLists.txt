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
  LIBRARIES New
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf tf2
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  New src/New.cpp
 )

add_executable(dwa src/the_node.cpp)

target_link_libraries(New ${catkin_LIBRARIES})
target_link_libraries(dwa New ${catkin_LIBRARIES})

