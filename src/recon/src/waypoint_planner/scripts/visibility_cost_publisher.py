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
from waypoint_planner.waypoint_planner_utils import SendMessage, GetMessage, rgb2gray, OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading
from matplotlib import image
import os
from collections import Counter

threading_lock = threading.Lock()

if __name__ == '__main__':
    rospy.init_node('visibility_lowres_to_highres_converter')
    working_dir = "/home/" + str(os.environ['USER']) + "/src/zone_recon/"

    robot_type = rospy.get_param("~robot_type")
    costmap = GetMessage(topic=rospy.get_param("~costmap_update_topic"), msg_type=OccupancyGrid)
    cell_size = rospy.get_param("~cell_size")
    
    visibility_prior = image.imread(working_dir + "assets/" + \
                                         rospy.get_param("~location") + "/visibility_cost_" + \
                                         rospy.get_param("~map_type") + ".png")
    visibility_prior = rgb2gray(visibility_prior)
    rospy.loginfo("Visibility prior dimensions " + str(visibility_prior.shape))
    visibility_prior /= np.max(visibility_prior)
    while not costmap.msg and robot_type == "ugv" and not rospy.is_shutdown():
        rospy.sleep(0.5)
        rospy.loginfo_throttle(10, "Waiting for costmap info")

    if robot_type == "ugv":
        n1 = int(np.ceil(
            costmap.msg.info.height * costmap.msg.info.resolution / cell_size))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
        n2 = int(np.ceil(
            costmap.msg.info.width * costmap.msg.info.resolution / cell_size))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
    if robot_type == "uas":
        n1 = int(np.ceil(rospy.get_param("~costmap_cell_size") * rospy.get_param("~map_height") / (
            cell_size)))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
        n2 = int(np.ceil(rospy.get_param("~costmap_cell_size") * rospy.get_param("~map_width") / (
            cell_size)))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size

    robot_name = rospy.get_param("~robot_name")
    lowres_topic = rospy.get_param("~viz_costmap_topic_lowres")
    highres_topic = rospy.get_param("~viz_costmap_topic")

    publisher = SendMessage(highres_topic, OccupancyGrid)
    rospy.loginfo("Grid-size:%dx%d with each cell %dmx%dm" % (n1, n2, cell_size, cell_size))

    print("Current namespace: ", rospy.get_namespace(), "should be like /recbot0 or recbot0")
    print("lowres topic: ", lowres_topic, " Should be aidtr_auto/local_pose")
    print("highres topic: ", highres_topic, " Should be robot_objects")


    def publish_when_you_get_it(float32multiarray_msg):
        # first convert to Robot object type message
        heatmap = OccupancyGrid()
        data_vector = np.array(float32multiarray_msg.data).flatten()
        rospy.loginfo("ZE DATA ZAT I HAF GOTTEN IZ: ")
        print(data_vector)
        if sum(data_vector) == 0:
            with_prior = 0.05 * np.flipud(visibility_prior)
        else:
            grid_multiplier = int(rospy.get_param("~cell_size") / rospy.get_param("~costmap_cell_size"))
            # try concat instead of repeat
            reorient_grid = np.repeat(np.repeat(np.array(np.transpose(data_vector.reshape((n2, n1)))),
                                                grid_multiplier, axis=0),
                                      grid_multiplier, axis=1)
            print("Shape of repeated grid: ", reorient_grid.shape)
            print("Shape needed: ", rospy.get_param("~map_height"), rospy.get_param("~map_width"), "or ",
                  costmap.msg.info.height, costmap.msg.info.width)
            with_prior = reorient_grid[:costmap.msg.info.height, :costmap.msg.info.width]
            with_prior[with_prior == 0] = 0.05
            with_prior = with_prior * np.flipud(visibility_prior)

        clipped_grid = 99 * with_prior / np.max(with_prior)
        # clipped_grid = 100 * np.flipud(visibility_prior)
        print("Shape of clipped grid: ", clipped_grid.shape)
        print("Max of clipped grid: ", np.max(clipped_grid), "min of clipped grid: ", np.min(clipped_grid))
        flat_repeat = clipped_grid.reshape(-1, 1).flatten()
        data = flat_repeat.astype(int)
        heatmap.data = list(data)
        # print("heatmap max: ", max(heatmap.data), " min: ", min(heatmap.data), "heatmap stats: ", Counter(heatmap.data))
        # print(publisher)
        heatmap.info.height = costmap.msg.info.height
        heatmap.info.width = costmap.msg.info.width
        heatmap.info.resolution = rospy.get_param("~costmap_cell_size")

        origin = Pose()
        origin.position.x = 0
        origin.position.y = 0
        origin.position.z = 0
        origin.orientation.x = 0
        origin.orientation.y = 0
        origin.orientation.z = 0
        origin.orientation.w = 1
        heatmap.info.origin = origin
        heatmap.header.stamp = rospy.Time.now()
        heatmap.header.frame_id = robot_name + rospy.get_param("~map_frame")
        publisher.msg = heatmap
        publisher.publish()
        return


    subscriber = GetMessage(lowres_topic, Float32MultiArray, callback_modifier=publish_when_you_get_it,
                            threading_lock=threading_lock)

    rospy.spin()
