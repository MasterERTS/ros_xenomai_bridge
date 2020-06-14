cmake_minimum_required(VERSION 2.8.3)
project(xddp_bringup)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)