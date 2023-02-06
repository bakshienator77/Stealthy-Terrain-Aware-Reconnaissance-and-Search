cmake_minimum_required(VERSION 3.0.2)
project(waypoint_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)
find_package(aidtr_msgs)



# So that catkin knows to read the setup.py
catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_planner
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(PROGRAMS
     scripts/waypoint_action_client
     scripts/convert_to_search_polygon.py
     scripts/simulate_threats.py
     scripts/evaluate_policies.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

