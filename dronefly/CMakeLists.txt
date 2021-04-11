cmake_minimum_required(VERSION 3.0.2)
project(dronefly)

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
  rospy
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
#  INCLUDE_DIRS include
#  LIBRARIES landingdrone
  CATKIN_DEPENDS angles eigen_conversions geometry_msgs mavros mavros_extras roscpp rospy std_msgs tf2_ros visualization_msgs
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
add_library(dronefly_lib
  src/lib/compute_pid.cpp
  src/lib/pid.cpp
  src/lib/callback.cpp
  src/lib/drone.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  dronefly_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(
  drone_run
  src/maincode/control.cpp
)

add_executable(
  markerpoint
  src/maincode/markerpoint.cpp
)

add_dependencies(
  drone_run
  markerpoint
  ## lib
  dronefly_lib
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  drone_run
  dronefly_lib
)

target_link_libraries(
  markerpoint
  dronefly_lib
)

## Mark executables and/or libraries for installation
install(TARGETS dronefly_lib drone_run markerpoint
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
  scripts/sendpoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
