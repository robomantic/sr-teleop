cmake_minimum_required(VERSION 2.8.3)
project(sr_remappers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  sr_robot_msgs
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include
LIBRARIES sr_remappers
CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs sr_robot_msgs trajectory_msgs sr_cyberglove_config
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_library(${PROJECT_NAME}
  src/shadowhand_to_cyberglove_remapper.cpp
  src/calibration_parser.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)



## Declare a cpp executable
add_executable(cyberglove_remapper
  src/shadowhand_to_cyberglove_remapper.cpp
  src/shadowhand_to_cyberglove_remapper_node.cpp
  src/calibration_parser.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(cyberglove_remapper
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(cyberglove_remapper
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(cyberglove_human_remapper
  src/humanhand_to_cyberglove_remapper.cpp
  src/humanhand_to_cyberglove_remapper_node.cpp
  src/calibration_parser.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(cyberglove_human_remapper
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(cyberglove_human_remapper
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
install(TARGETS ${PROJECT_NAME} cyberglove_remapper cyberglove_human_remapper
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES include/${PROJECT_NAME}/calibration_parser.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
catkin_install_python(PROGRAMS scripts/js_to_jtp.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
