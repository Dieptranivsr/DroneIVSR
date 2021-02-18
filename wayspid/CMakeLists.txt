cmake_minimum_required(VERSION 3.0.2)
project(wayspid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#find_package(gazebo REQUIRED)
#find_package(cmake_modules REQUIRED)
#find_package(Eigen REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/setTrajectory.cfg
  cfg/setPID.cfg
  cfg/setSafeD9.cfg
  #...
)

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
#  INCLUDE_DIRS include
#  LIBRARIES wayspid
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs mavros_msgs roscpp roslib rospy sensor_msgs std_msgs tf message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include 
#   "/opt/ros/melodic/include"
   ${catkin_INCLUDE_DIRS} #${GAZEBO_INCLUDE_DIRS} #${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/wayspid.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/wayspid_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


add_executable(pid src/PID.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})
#add_dependencies(PID wayspid_generate_message_cpp wayspid_gencfg)
add_dependencies(pid wayspid_gencfg)

add_executable(fake_mocap_px4 src/fakeMocapPx4.cpp)
target_link_libraries(fake_mocap_px4 ${catkin_LIBRARIES})
#add_dependencies(fake_mocap_px4 controllers_generate_messages_cpp)

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
#add_dependencies(trajectory wayspid_generate_message_cpp wayspid_gencfg)
add_dependencies(trajectory wayspid_gencfg)

add_executable(safe_d9 src/safeD9.cpp)
target_link_libraries(safe_d9 ${catkin_LIBRARIES})
#add_dependencies(PID wayspid_generate_message_cpp wayspid_gencfg)
add_dependencies(safe_d9 wayspid_gencfg)
