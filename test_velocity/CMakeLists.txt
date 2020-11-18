cmake_minimum_required(VERSION 3.0.2)
project(test_velocity)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  geometry_msgs
  mavros
  mavros_extras
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_velocity
  CATKIN_DEPENDS angles geometry_msgs mavros mavros_extras roscpp std_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  diep_test_velocity
  src/lib/test_lib.cpp
  src/lib/pbPlots.cpp
  src/lib/supportLib.cpp
)

target_link_libraries(
  diep_test_velocity 
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(
  testvel_node
  src/test_vel.cpp
)

add_dependencies(
  testvel_node
  diep_test_velocity
)

target_link_libraries(
  testvel_node
  diep_test_velocity
)

## Mark executables and/or libraries for installation
install(TARGETS diep_test_velocity testvel_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
