cmake_minimum_required(VERSION 2.8.3)
project(delta_arduino)


find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
  message_generation
)
add_message_files(
 DIRECTORY msg
 FILES cmdAngle.msg

)
add_service_files(
 DIRECTORY srv
 FILES GetInfo.srv

)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime std_msgs
)


rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

rosserial_add_client_target(firmware delta_arduino ALL)
rosserial_add_client_target(firmware delta_arduino-upload)

