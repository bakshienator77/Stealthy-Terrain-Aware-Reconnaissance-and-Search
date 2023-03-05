
cmake_minimum_required(VERSION 3.10)

project(anon_ros_msgs  VERSION 0.0.1 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs)
find_package(genmsg REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/msg
   FILES
   KvhData.msg
   RobotObject.msg
   LastDetection.msg
   LocalTrack.msg
   FusionMsg.msg
   TrackData.msg
   TrackDataVector.msg
   SearchPolygon.msg
   Ins.msg
   BodyVelocity.msg
   InstallationAlignment.msg
   RobotGoal.msg
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
   CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs
)


