"""
Code for the work:

`` Stealthy Terrain-Aware Multi-Agent Active Search``,
Nikhil Angad Bakshi and Jeff Schneider
Robotics Institute, Carnegie Mellon University

(C) Nikhil Angad Bakshi 2023 (nabakshi@cs.cmu.edu)
Please cite the following paper to use the code:


@inproceedings{
bakshi2023stealthy,
title={Stealthy Terrain-Aware Multi-Agent Active Search},
author={Nikhil Angad Bakshi and Jeff Schneider},
booktitle={7th Annual Conference on Robot Learning},
year={2023},
url={https://openreview.net/forum?id=eE3fsO5Mi2}
}
"""
#!/usr/bin/env python


import rospy
import sys

from geometry_msgs.msg import PolygonStamped
from anon_ros_msgs.msg import SearchPolygon

rospy.init_node('convert_to_search_polygon')
robot_name = rospy.get_param("~robot_name")

pub_topic_name = rospy.get_param("~search_polygon_topic")
pub = rospy.Publisher(pub_topic_name, SearchPolygon)

rate = rospy.Rate(10)

def convertCallback(msg):
    out_msg = SearchPolygon()
    out_msg.polygon = msg.polygon
    out_msg.header = msg.header
    out_msg.zone_id = rospy.get_param("~zone_id", 0)
    out_msg.robot_id = robot_name
    pub.publish(out_msg)
    


def listener():   
    sub_topic_name = rospy.get_param("~atak_polygon_topic")
    rospy.Subscriber(sub_topic_name, PolygonStamped, convertCallback)
    rospy.spin()



if __name__ == '__main__':
    try:
        listener()
                
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)
