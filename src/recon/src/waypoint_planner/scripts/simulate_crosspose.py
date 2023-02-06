#!/usr/bin/env python


import rospy
from waypoint_planner.waypoint_planner_utils import SendMessage, GetMessage, transform_pose
from nav_msgs.msg import Odometry
from aidtr_ros_msgs.msg import RobotObject
import threading

threading_lock = threading.Lock()

if __name__ == '__main__':
    rospy.init_node('crosspose')
    """
    Need to get the local pose from other robots (Odometry format)
    How to get it from /<all namespaces except mine>/aidtr_auto/local_pose ?
    convert to RobotObject format
    Publish in current namespace /robot_object
    """
    robot_name = rospy.get_param("~robot_name")
    robot_object_topic = rospy.get_param("~cross_pose_topic")
    publisher = SendMessage(robot_object_topic, RobotObject)
    publisher.msg = RobotObject()
    publisher.publish()
    rospy.sleep(10)
    print("SIMULATING CROSSPOSE FOR ROBOT: ", robot_name)
    all_topics = rospy.get_published_topics()
    current_ns = rospy.get_namespace()
    local_pose_topic = rospy.get_param("~local_pose_topic")
    map_frame = rospy.get_param("~map_frame")

    print("Current namespace: ", rospy.get_namespace(), "should be like /recbot0 or recbot0")
    print("Local pose topic: ", local_pose_topic, " Should be aidtr_auto/local_pose")
    print("robot object topic: ", robot_object_topic, " Should be robot_objects")
    robot_object_topics = [topic[0] for topic in all_topics if (current_ns not in topic[0] and robot_object_topic in topic[0])]

    publishers = [SendMessage(topic, RobotObject) for topic in robot_object_topics]
    print("publishers: ", publishers)
    def publish_when_you_get_it(odometry_msg):
        # first convert to Robot object type message
        msg = RobotObject()
        msg.header = odometry_msg.header
        if rospy.get_param("~autonomy_faker"):
            msg.pose = odometry_msg.pose.pose
            msg.header.frame_id = rospy.get_param("~map_frame") #"earth"
        else:
            msg.pose = transform_pose(odometry_msg.pose.pose, msg.header.frame_id, "earth")
            if msg.pose is None:
                return
            msg.header.frame_id = "earth"
        # anything bad if the robot_id isn't set?
        msg.robot_id = robot_name
        # print(msg)
        [p.pub.publish(msg) for p in publishers]
        return



    subscriber = GetMessage(local_pose_topic, Odometry, callback_modifier=publish_when_you_get_it, threading_lock=threading_lock)
    
    rospy.spin()
