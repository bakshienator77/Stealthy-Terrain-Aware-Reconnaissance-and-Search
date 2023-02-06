cmake_minimum_required(VERSION 3.0.2)
project(zone_recon_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs nav_msgs geometry_msgs)
find_package(genmsg REQUIRED)

add_message_files(
    DIRECTORY msg
  FILES
    TrackData.msg
    TrackDataVector.msg
    ThreatVector.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_planner
  CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs actionlib_msgs message_runtime
#  DEPENDS system_lib
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

ADD_LIBRARY(${PROJECT_NAME} INTERFACE)

