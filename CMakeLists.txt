cmake_minimum_required(VERSION 2.8.3)
project(iirob_led)

#TODO Remove when testing is done
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -Wall -s -O3 -g0") #-Os

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
  geometry_msgs
)

add_service_files(
  FILES
  SetLedDirectory.srv
)

add_message_files(
  FILES
  DirectionWithForce.msg
  TurnLedsOnOff.msg
)

add_action_files(
  FILES
  Blinky.action
  Police.action
  FourRegions.action
  ChaserLight.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS common/LED common/SERIAL common/RGBCONVERTER include/iirob_led
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS actionlib actionlib_msgs message_runtime roscpp std_msgs geometry_msgs
 DEPENDS system_lib
)

include_directories(
  common/LED
  common/SERIAL
  common/RGBCONVERTER
  include/iirob_led
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DIIROB_EXPORT=)

###########
## Build ##
###########
# The ledcommon library contains all that is required to control the underlying hardware
add_library(ledcommon
  common/LED/LEDStrip.cpp
  common/LED/LEDModul.cpp
  common/SERIAL/SerialPort.cpp
  common/SERIAL/PosixSerialPort.cpp
  common/SERIAL/SerialPortFactory.cpp
)
# The rgbconverter contains methods for manipulating colour data
add_library(rgbconverter common/RGBCONVERTER/RGBConverter.cpp)
# The iirob_led_base contains functionality that is used by others
add_library(iirob_led_base src/iirob_led_base.cpp)
target_link_libraries(iirob_led_base rgbconverter ledcommon ${catkin_LIBRARIES})
# The iirob_led_rectangular contains functionality for controlling the SR2 LED rectangular strip using ROS
add_library(iirob_led_rectangle src/iirob_led_rectangle.cpp)
target_link_libraries(iirob_led_rectangle iirob_led_base)
# The iirob_led_cross contains functionality for controlling the SR2 LED cross strip using ROS
add_library(iirob_led_cross src/iirob_led_cross.cpp)
target_link_libraries(iirob_led_cross iirob_led_base)

add_executable(${PROJECT_NAME}_node src/iirob_led_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${PROJECT_NAME}_node iirob_led_rectangle iirob_led_cross ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node iirob_led_rectangle iirob_led_cross ${catkin_LIBRARIES})

# Playground using an action client (old version, may need update). Code is to be used only as an example
#add_executable(${PROJECT_NAME}_node_client_test src/iirob_led_node_client_test.cpp)
#add_dependencies(${PROJECT_NAME}_node_client_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${PROJECT_NAME}_node_client_test
#  ${PROJECT_NAME}
#  rgbconverter
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

install(TARGETS ledcommon rgbconverter iirob_led_rectangle iirob_led_cross ${PROJECT_NAME}_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_iirob_led.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
