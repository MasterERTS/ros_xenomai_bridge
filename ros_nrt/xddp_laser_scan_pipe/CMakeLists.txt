cmake_minimum_required(VERSION 2.8.3)
project(xddp_laser_scan_pipe)
find_package(catkin REQUIRED COMPONENTS
  xddp_utilities
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xddp_laser src/xddp_laser.cpp)
target_link_libraries(xddp_laser ${catkin_LIBRARIES})
