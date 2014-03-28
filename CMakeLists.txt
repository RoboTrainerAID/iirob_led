cmake_minimum_required(VERSION 2.8.3)
project(iirob_led)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  FILES
  PlayLed.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS common/LED common/SERIAL
 LIBRARIES iirob_led
 CATKIN_DEPENDS message_runtime roscpp std_msgs actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  common/LED
  common/SERIAL
  ${catkin_INCLUDE_DIRS}
)

add_definitions( -DIROB_EXPORT= )

## Declare a cpp library
add_library(iirob_led
  common/LED/LEDStrip.cpp
  common/LED/LEDModul.cpp
  common/SERIAL/SerialPort.cpp
  common/SERIAL/PosixSerialPort.cpp
  common/SERIAL/SerialPortFactory.cpp
)

add_executable(iirob_led_node src/iirob_led_node.cpp)
add_dependencies(iirob_led_node iirob_led_generate_messages_cpp)
target_link_libraries(iirob_led_node
  iirob_led
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS iirob_led iirob_led_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY common/include
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
#
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)