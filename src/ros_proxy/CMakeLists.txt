cmake_minimum_required(VERSION 3.4.1)
project(ros_proxy)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  #traj_utils
  controller_msgs
  apriltag_ros
  visualization_msgs
)

catkin_package(
   LIBRARIES
   CATKIN_DEPENDS roscpp  sensor_msgs std_msgs nav_msgs geometry_msgs sensor_msgs controller_msgs apriltag_ros#traj_utils
   DEPENDS EIGEN3
)


include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(CameraPoseVisualization
  include/CameraPoseVisualization.cpp
) 
target_link_libraries(CameraPoseVisualization ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} )

add_executable(ros_proxy
    src/ros_proxy.cpp 
    )
target_link_libraries(ros_proxy
    ${catkin_LIBRARIES}
    nanomsg
    )

add_executable(ros_proxy_safmc
    src/ros_proxy_safmc.cpp 
    )
target_link_libraries(ros_proxy_safmc
    ${catkin_LIBRARIES}
    nanomsg
    CameraPoseVisualization
    )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME} usb_cam
install(TARGETS ${PROJECT_NAME} ros_proxy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} ros_proxy_safmc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

