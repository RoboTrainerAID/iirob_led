cmake_minimum_required(VERSION 2.8.3)
project(iirob_led)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  std_msgs
)

add_service_files(
  FILES
  SetLedDirectory.srv
)

add_action_files(
  FILES
  PlayLed.action            # ???
  Blinky.action             # action 2
  Police.action             # action 3
  FourMusketeers.action     # action 4
  RunningBunny.action       # action 5
  Spinner.action            # action 6
  Changeling.action         # action 7
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS common/LED common/SERIAL
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS actionlib actionlib_msgs message_runtime roscpp std_msgs
 DEPENDS system_lib
)

include_directories(
  common/LED
  common/SERIAL
  common/RGBCONVERTER
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DIIROB_EXPORT=)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  common/LED/LEDStrip.cpp
  common/LED/LEDModul.cpp
  common/SERIAL/SerialPort.cpp
  common/SERIAL/PosixSerialPort.cpp
  common/SERIAL/SerialPortFactory.cpp
)

add_library(rgbconverter common/RGBCONVERTER/RGBConverter.cpp)

add_executable(${PROJECT_NAME}_node src/iirob_led_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  #rgbconverter
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node_client_test src/iirob_led_node_client_test.cpp)
add_dependencies(${PROJECT_NAME}_node_client_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node_client_test
  ${PROJECT_NAME}
  rgbconverter
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
