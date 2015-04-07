cmake_minimum_required(VERSION 2.8.3)
project(cyberglove)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  std_msgs
  sensor_msgs
  diagnostic_msgs
  genmsg
  sr_cyberglove_config
  cereal_port
  message_generation
)

find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM ${Boost_INCLUDE_DIR})

add_service_files(
  FILES
  Calibration.srv
  Start.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  diagnostic_msgs
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES cyberglove
CATKIN_DEPENDS roslib roscpp rospy std_msgs sensor_msgs diagnostic_msgs genmsg sr_cyberglove_config cereal_port message_runtime
#  DEPENDS system_lib
)


add_library(cyberglove
  src/cyberglove_publisher.cpp
  src/serial_glove.cpp
  src/xml_calibration_parser.cpp
  src/cyberglove_service.cpp
)

add_dependencies(cyberglove
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(cyberglove
#  cereal_port
  tinyxml
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


## Declare a cpp executable
add_executable(cyberglove_node
  src/cyberglove_publisher.cpp
  src/cyberglove_node.cpp
  src/serial_glove.cpp
  src/xml_calibration_parser.cpp
  src/cyberglove_service.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(cyberglove_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(cyberglove_node
#  cereal_port
  tinyxml
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS cyberglove
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/cyberglove/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest_gtest(test_cyberglove
    test/test_calibration.test
    test/test_calibration.cpp
    src/xml_calibration_parser.cpp
  )
  target_link_libraries(test_cyberglove
    tinyxml
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${Boost_LIBRARIES}
  )

endif()
