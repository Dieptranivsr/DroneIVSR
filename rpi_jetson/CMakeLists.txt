cmake_minimum_required(VERSION 3.0.2)
project(rpi_jetson)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rpi_jetson
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rpi_side src/rpi_side.cpp)
target_link_libraries(rpi_side ${catkin_LIBRARIES})

add_executable(jetson_side src/jetson_side.cpp)
target_link_libraries(jetson_side ${catkin_LIBRARIES})

add_executable(push_message src/push_message.cpp)
target_link_libraries(push_message ${catkin_LIBRARIES})

add_executable(wait_message src/wait_message.cpp)
target_link_libraries(wait_message ${catkin_LIBRARIES})
