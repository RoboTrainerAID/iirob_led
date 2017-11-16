cmake_minimum_required(VERSION 2.8.3)
project(iirob_led)

# Set this if you need to test
#set(CMAKE_BUILD_TYPE Debug)   # Release

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")   # This is important! See position independed code (->Google)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -Wall -s -O3 -g0")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" Cpp11CompilerFlag)
if (Cpp11CompilerFlag)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_service_files(
  FILES
  SetLedDirectory.srv
  TurnOnOff.srv
)

add_message_files(
  FILES
  ForceWithColor.msg
  SetLedRange.msg
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
 INCLUDE_DIRS common/LED common/SERIAL common/RGBCONVERTER include
 LIBRARIES ${PROJECT_NAME}_base ${PROJECT_NAME}_rectangle ${PROJECT_NAME}_cross
 CATKIN_DEPENDS actionlib actionlib_msgs message_runtime roscpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
 # DEPENDS 
)

include_directories(
  common/LED
  common/SERIAL
  common/RGBCONVERTER
  common/MATH
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DIIROB_EXPORT=)

###########
## Build ##
###########
# The ${PROJECT_NAME}_ledcommon library contains all that is required to control the underlying hardware
add_library(${PROJECT_NAME}_ledcommon
  common/LED/LEDStrip.cpp
  common/LED/LEDModul.cpp
  common/SERIAL/SerialPort.cpp
  common/SERIAL/PosixSerialPort.cpp
  common/SERIAL/SerialPortFactory.cpp
)
# The ${PROJECT_NAME}_rgbconvert contains methods for manipulating colour data
add_library(${PROJECT_NAME}_rgbconvert common/RGBCONVERTER/RGBConverter.cpp)
# The iirob_led_math contains various math-related entities
add_library(${PROJECT_NAME}_math common/MATH/iirob_led_math.cpp)
# The iirob_led_base contains functionality that is used by others
add_library(${PROJECT_NAME}_base src/iirob_led_base.cpp)
add_dependencies(${PROJECT_NAME}_base ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_base ${PROJECT_NAME}_rgbconvert ${PROJECT_NAME}_ledcommon ${PROJECT_NAME}_math ${catkin_LIBRARIES})
# The iirob_led_rectangular contains functionality for controlling the SR2 LED rectangular strip using ROS
add_library(${PROJECT_NAME}_rectangle src/iirob_led_rectangle.cpp)
add_dependencies(${PROJECT_NAME}_rectangle ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_rectangle ${PROJECT_NAME}_base)
# The iirob_led_cross contains functionality for controlling the SR2 LED cross strip using ROS
add_library(${PROJECT_NAME}_cross src/iirob_led_cross.cpp)
add_dependencies(${PROJECT_NAME}_cross ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_cross ${PROJECT_NAME}_base)

add_executable(${PROJECT_NAME}_node src/iirob_led_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${PROJECT_NAME}_node iirob_led_rectangle iirob_led_cross ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_rectangle ${PROJECT_NAME}_cross ${catkin_LIBRARIES})

# Playground using an action client (old version, may need update). Code is to be used only as an example
#add_executable(${PROJECT_NAME}_node_client_test src/iirob_led_node_client_test.cpp)
#add_dependencies(${PROJECT_NAME}_node_client_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${PROJECT_NAME}_node_client_test
#  ${PROJECT_NAME}
#  ${PROJECT_NAME}_rgbconvert
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_ledcommon ${PROJECT_NAME}_rgbconvert ${PROJECT_NAME}_rectangle ${PROJECT_NAME}_cross ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

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
