cmake_minimum_required(VERSION 2.8.3)
project(state_pcl)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3.2.0 REQUIRED)       # if you install the latest version of OpenCV LIBRARIES

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)
add_definitions(${PCL_DEFINITIONS})

# turn off WARNINGS Notification
# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

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

  quadrotor_msgs
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

#add_executable(node_state
#  src/node_state.cpp
#  )

#add_executable(show_lidar             # node show_lidar
#  src/show_lidar.cpp
#  )

#add_executable(depth_pcl
#  src/depth_pcl.cpp
#  )

add_executable(visual_traj
  src/visual_trajectory.cpp
  )

#target_link_libraries(node_state
#  ${catkin_LIBRARIES}
#  )

#target_link_libraries(show_lidar      # node show_lidar
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  )

#target_link_libraries(depth_pcl
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  )

target_link_libraries(visual_traj
  ${catkin_LIBRARIES}
  )

#catkin_install_python(PROGRAMS scripts/getDepth.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
