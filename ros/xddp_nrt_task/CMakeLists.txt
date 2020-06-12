cmake_minimum_required(VERSION 2.8.3)
project(xddp_nrt_task)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
)
include_directories(
  include
  classes
  ${catkin_INCLUDE_DIRS}
)

add_library(listen classes/ListenerXddp.cpp)
add_library(chat classes/ChatterXddp.cpp)

add_executable(xddp_chatter src/chatter.cpp)
add_executable(xddp_listener src/listener.cpp)
target_link_libraries(xddp_chatter chat ${catkin_LIBRARIES})
target_link_libraries(xddp_listener listen ${catkin_LIBRARIES})