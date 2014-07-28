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
)

find_package(Boost REQUIRED system filesystem date_time thread)

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
  LIBRARIES
  CATKIN_DEPENDS
    roslib
    roscpp
    rospy
    std_msgs
    sensor_msgs
    diagnostic_msgs
    genmsg
    sr_cyberglove_config
  DEPENDS
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cyberglove
  src/cyberglove_publisher.cpp
  src/cyberglove_node.cpp
  src/serial_glove.cpp
  src/xml_calibration_parser.cpp
  src/cyberglove_service.cpp
)

add_dependencies(cyberglove
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(cyberglove
  tinyxml
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS cyberglove
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(COMMAND add_rostest_gtest)
    add_rostest_gtest(test_cyberglove
        test/test_calibration.test
        test/test_calibration.cpp
        src/xml_calibration_parser.cpp
    )
    add_dependencies(test_cyberglove ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_cyberglove
        tinyxml
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES}
        ${Boost_LIBRARIES}
    )
endif(COMMAND add_rostest_gtest)

