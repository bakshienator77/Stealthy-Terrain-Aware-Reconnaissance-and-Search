cmake_minimum_required(VERSION 3.10.2...3.24.1)
project(anon_msgs)
SET(CMAKE_CXX_FLAGS "-std=c++17 -O2 -g -Wall -Werror -Wextra ${CMAKE_CXX_FLAGS}")

# Allow turning on and off werror
OPTION(WARNINGS_AS_ERROR "Treat compiler warnings as error" OFF)

# Various compiler flags
# src
get_filename_component(SRC_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
message("src dir: ${SRC_DIR}")
# project root
get_filename_component(AUTONOMY_DIR ${SRC_DIR} DIRECTORY)
message("Autonomy dir: ${AUTONOMY_DIR}")
INCLUDE(${AUTONOMY_DIR}/cmake/CompilerFlags.cmake)

find_package(
  catkin REQUIRED COMPONENTS 
    actionlib_msgs
    roscpp
    message_generation
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
)

add_service_files(
  FILES
  LocalPose.srv
  PtzTrigger.srv
)

add_action_files(
  DIRECTORY action
  FILES Autonomy.action
)

add_message_files(
    DIRECTORY msg
  FILES
    SafetyMgrState.msg
    VehicleStatus.msg
    TrackSpeed.msg
    RcvFeedback.msg
    RecbotFeedback.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs  
    roscpp
    message_runtime
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
)
