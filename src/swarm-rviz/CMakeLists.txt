cmake_minimum_required(VERSION 3.0.2) 
project(swarm_rviz)

add_definitions(-std=c++14)     
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS rviz
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  controller_msgs
)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)  

add_definitions(-DQT_NO_KEYWORDS)

# Generate messages in the 'msg' folder
#add_message_files(
#FILES
#cmd.msg
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs controller_msgs
)

include_directories(SYSTEM include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#include_directories(/opt/ros/noetic/include)

qt5_wrap_cpp(SWARM_FILES
  src/swarm_tool.h
)

set(SOURCE_FILES
  src/swarm_tool.cpp
  ${SWARM_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
qt5_use_modules(${PROJECT_NAME} Core)        #may be not need
qt5_use_modules(${PROJECT_NAME} Widgets)   #may be not need
qt5_use_modules(${PROJECT_NAME} Gui)           #may be not need

target_link_libraries(${PROJECT_NAME}
   librviz.so librviz_default_plugin.so 
   ${QT_LIBRARIES} ${catkin_LIBRARIES})
                        

install(TARGETS
   ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
   config/plugin_description.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
