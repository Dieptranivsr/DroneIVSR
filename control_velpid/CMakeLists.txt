cmake_minimum_required(VERSION 3.0.2)
project(control_velpid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES control_velpid
  CATKIN_DEPENDS angles eigen_conversions geometry_msgs mavros mavros_extras roscpp std_msgs tf2_ros visualization_msgs
  DEPENDS Boost
)

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
  src/origin/ctrl_vel.cpp
)

add_executable(
  square_velpid_node 
  src/square_ctrl_vel.cpp
)

#add_executable(
#  circle_velpid_node 
#  src/sitl/circle_ctrl_vel.cpp
#)

add_executable(
  control_v2
  src/latest/controlv2.cpp
)

add_executable(
  control_worldframe
  src/latest/control_worldframe.cpp
)

add_executable(
  square_path
  src/latest/square_path.cpp             # have trajectory
)

add_executable(
  square_notrajectory
  src/latest/one2fourpoints.cpp          # no trajectory
)

############ HITL ###############
add_executable(
  hitl_point2point
  src/hitl/points.cpp
)

add_executable(
  hitl_squarepath
  src/hitl/square.cpp
)

add_executable(
  hitl_squarenotrajectory
  src/hitl/hitlone2fourpoints.cpp
)

#################################

add_dependencies(
  # SITL
  control_velpid_node
  square_velpid_node
  control_v2
  control_worldframe
  square_path
  square_notrajectory
  #circle_velpid_node
  # HITL
  hitl_point2point
  hitl_squarepath
  hitl_squarenotrajectory
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
  square_notrajectory
  control_velpid_lib
)

# HITL
target_link_libraries(
  hitl_point2point
  control_velpid_lib
)

target_link_libraries(
  hitl_squarepath
  control_velpid_lib
)

target_link_libraries(
  hitl_squarenotrajectory
  control_velpid_lib
)

## Mark executables and/or libraries for installation
install(TARGETS control_velpid_lib control_velpid_node square_velpid_node square_path control_worldframe hitl_point2point hitl_squarepath square_notrajectory hitl_squarenotrajectory #circle_velpid_node control_v2 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY launch/sitl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/sitl
)

install(DIRECTORY launch/hitl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/hitl
)

catkin_install_python(PROGRAMS
  scripts/getResults.py
  scripts/getRs2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
