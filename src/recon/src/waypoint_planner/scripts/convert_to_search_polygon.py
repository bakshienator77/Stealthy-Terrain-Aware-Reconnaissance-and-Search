#!/usr/bin/env python

# Copyright 2020
#
# National Robotics Engineering Center, Carnegie Mellon University
# 10 40th Street, Pittsburgh, PA 15201
# www.rec.ri.cmu.edu
#
# Control Instructions (who can see this code):
# NREC Confidential.  Not for public release unless permission granted
# by program manager.
#
# Usage Rights (who can use this code):
# Usage allowed for all NREC programs with permissions from author and
# program manager.
#
# This notice must appear in all copies of this file and its derivatives.
#
# Created under Program: AIDTR
#

import rospy
import sys

from geometry_msgs.msg import PolygonStamped
from aidtr_ros_msgs.msg import SearchPolygon

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
