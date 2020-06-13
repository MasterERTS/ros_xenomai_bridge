cmake_minimum_required(VERSION 2.8.3)
project(xddp_talker)
find_package(catkin REQUIRED COMPONENTS
  xddp_utilities
  roscpp
  std_msgs
)
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    xddp_utilities
    roscpp
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xddp_chatter src/chatter_node.cpp)
add_executable(nrt_thread src/xddp_nrt_thread_node.cpp)
add_executable(xddp_listener src/listener_node.cpp)

add_dependencies(xddp_chatter ${catkin_EXPORTED_TARGETS})
add_dependencies(nrt_thread ${catkin_EXPORTED_TARGETS})
add_dependencies(xddp_listener ${catkin_EXPORTED_TARGETS})

target_link_libraries(xddp_chatter ${catkin_LIBRARIES})
target_link_libraries(xddp_listener ${catkin_LIBRARIES})
target_link_libraries(nrt_thread ${catkin_LIBRARIES})
