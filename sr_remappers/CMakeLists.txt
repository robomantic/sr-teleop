cmake_minimum_required(VERSION 2.8.3)
project(sr_remappers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  sr_robot_msgs
  sr_cyberglove_config
  cyberglove
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    sr_robot_msgs
    sr_cyberglove_config
    cyberglove
  DEPENDS
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cyberglove_remapper
  src/shadowhand_to_cyberglove_remapper.cpp
  src/shadowhand_to_cyberglove_remapper_node.cpp
  src/calibration_parser.cpp
)
add_dependencies(cyberglove_remapper
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cyberglove_remapper
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(leaphand_cyberglove_remapper
  src/leaphand_to_cyberglove_remapper.cpp
  src/leaphand_to_cyberglove_remapper_node.cpp
  src/calibration_parser.cpp
)
add_dependencies(leaphand_cyberglove_remapper
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(leaphand_cyberglove_remapper
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS cyberglove_remapper
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
