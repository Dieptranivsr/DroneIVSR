cmake_minimum_required(VERSION 3.0.2)
project(offboardpy)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  #pyquaternion
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboardpy
#  CATKIN_DEPENDS geometry_msgs mavros_msgs pyquaternion rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/pythongaas/commander.py
  scripts/pythongaas/mavros_run.py
  scripts/pythongaas/fly_circle.py
  # SITL
  scripts/sitl/offb_node.py
  # HITL
  scripts/hitl/hitl_offb.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

