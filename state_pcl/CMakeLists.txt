cmake_minimum_required(VERSION 2.8.3)
project(state_pcl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  message_filters

  cv_bridge
  sensor_msgs
  image_geometry
  image_transport
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
 LIBRARIES state_pcl
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(node_state
  src/node_state.cpp
  )

#add_executable(check_lidar           #ignore
#  src/node_lidar.cpp
#  )

add_executable(show_lidar
  src/show_lidar.cpp
  )

add_executable(depth_pcl
  src/depth_pcl.cpp
  )

target_link_libraries(node_state
  ${catkin_LIBRARIES}
  )

#target_link_libraries(check_lidar    #ignore
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  )

target_link_libraries(show_lidar
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries(depth_pcl
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

#catkin_install_python(PROGRAMS scripts/getDepth.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
