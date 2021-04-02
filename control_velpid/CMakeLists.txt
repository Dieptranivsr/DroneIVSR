cmake_minimum_required(VERSION 3.0.2)
project(control_velpid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  eigen_conversions
  geometry_msgs
  mavros
  mavros_extras
  roscpp
  std_msgs
  tf2_ros
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES control_velpid
  CATKIN_DEPENDS angles eigen_conversions geometry_msgs mavros mavros_extras roscpp std_msgs tf2_ros visualization_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(control_velpid_lib
  src/lib/velpid_lib.cpp
  src/lib/pid_controller.cpp
  src/lib/pid_controllerv2.cpp
  src/lib/captureGraph.cpp
  src/lib/pbPlots.cpp
  src/lib/supportLib.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  control_velpid_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(
  control_velpid_node 
  src/ctrl_vel.cpp
)

add_executable(
  square_velpid_node 
  src/square_ctrl_vel.cpp
)

#add_executable(
#  circle_velpid_node 
#  src/circle_ctrl_vel.cpp
#)

add_executable(
  control_v2
  src/controlv2.cpp
)

add_executable(
  control_worldframe
  src/control_worldframe.cpp
)

add_executable(
  square_path
  src/square_path.cpp
)

############ HITL ###############
add_executable(
  point2point
  src/hitl/points.cpp
)

add_executable(
  squarepath
  src/hitl/square.cpp
)
#################################

add_dependencies(
  # SITL
  control_velpid_node
  square_velpid_node
  control_v2
  control_worldframe
  square_path
  #circle_velpid_node
  # HITL
  point2point
  squarepath
  control_velpid_lib
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  control_velpid_node
  control_velpid_lib
)

target_link_libraries(
  square_velpid_node
  control_velpid_lib
)

#target_link_libraries(
#  circle_velpid_node
#  control_velpid_lib
#)

target_link_libraries(
  control_v2
  control_velpid_lib
)

target_link_libraries(
  control_worldframe
  control_velpid_lib
)

target_link_libraries(
  square_path
  control_velpid_lib
)

target_link_libraries(
  point2point
  control_velpid_lib
)

target_link_libraries(
  squarepath
  control_velpid_lib
)

## Mark executables and/or libraries for installation
install(TARGETS control_velpid_lib control_velpid_node square_velpid_node square_path control_worldframe point2point squarepath #circle_velpid_node control_v2 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
  scripts/getResults.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
