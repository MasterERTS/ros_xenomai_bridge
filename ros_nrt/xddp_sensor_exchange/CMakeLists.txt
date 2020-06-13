cmake_minimum_required(VERSION 2.8.3)
project(xddp_sensor_exchange)
find_package(catkin REQUIRED COMPONENTS
  xddp_utilities
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_exchange src/sensor_exchange.cpp)
target_link_libraries(sensor_exchange ${catkin_LIBRARIES})
