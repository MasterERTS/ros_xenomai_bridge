cmake_minimum_required(VERSION 2.8.3)
project(xddp_sensors)
find_package(catkin REQUIRED COMPONENTS
  xddp_utilities
  roscpp
  nav_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xddp_sensors src/xddp_sensors.cpp)
target_link_libraries(xddp_sensors ${catkin_LIBRARIES})
