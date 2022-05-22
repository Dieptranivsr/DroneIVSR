cmake_minimum_required(VERSION 2.8.3)
project(check_state)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
#find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  message_filters
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DroneState.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
# INCLUDE_DIRS include
 LIBRARIES test_sync
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)


#add_executable(test_sync
#  src/sync_node.cpp
#  )

add_executable(node_state
  src/node_state.cpp
  )

#target_link_libraries(test_sync
#  ${catkin_LIBRARIES}
#  )

target_link_libraries(node_state
  ${catkin_LIBRARIES}
  )

#catkin_install_python(PROGRAMS scripts/getDepth.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
