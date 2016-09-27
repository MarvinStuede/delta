cmake_minimum_required(VERSION 2.8.3)

add_executable(main main.cpp)

include_directories(
  ${ROS_LIB_DIR}
  )
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/libraries

  )
link_directories(/home/marvin/catkin_ws/src/delta/delta_arduino/include/arduino)
# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)

generate_arduino_firmware(delta_arduino
  SRCS main.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega2560
  PORT /dev/arduino
)
