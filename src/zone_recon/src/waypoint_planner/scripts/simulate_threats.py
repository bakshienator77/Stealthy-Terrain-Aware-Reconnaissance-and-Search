#!/usr/bin/env python

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

from __future__ import print_function

import rospy
import sys
import uuid
import tf2_ros
import threading
import copy
import time
import numpy as np
import matplotlib.pyplot as plt 
from matplotlib import cm

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped
from map_msgs.msg import OccupancyGridUpdate
from anon_ros_msgs.msg import SearchPolygon, RobotObject
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from zone_recon_msgs.msg import TrackDataVector, TrackData, ThreatVector

from waypoint_planner.waypoint_planner_utils import GetMessage, SendMessage, transform_pose, is_within_search_region, \
    grid2map, start2map, map2grid, euler_from_quaternion, rgb2gray, compute_probability_prior, quaternion_to_facing_direction
import pickle as pkl
import os
from matplotlib import cm, image
import threading
from PIL import Image

threading_lock = threading.Lock()

# copied from active_search.py
def create_directional_sensor(noise_vec, n1, n2, l, h, d):
    n = n1*n2 
    
    robot_type = rospy.get_param("~robot_type")
    if robot_type == "uas":
        exit("threat simulation has not been implemented for UAS")

    if rospy.get_param("~fov_setting") == "conservative":
        viewsheds = {"l": [[l, l], [l + 1, l + 2], [l, l], [l - 1, l - 2]],
                     "h": [[h + 1, h + 2], [h, h], [h - 1, h - 2], [h, h]]}  # up, right, down, left
    elif rospy.get_param("~fov_setting") == "viewshed":
        viewsheds = {
            "l": [[l, l - 1, l, l + 1, l - 2, l - 1, l, l + 1, l + 2, l - 3, l - 2, l - 1, l, l + 1, l + 2, l + 3],
                  [l, l + 1, l + 1, l + 1, l + 2, l + 2, l + 2, l + 2, l + 2, l + 3, l + 3, l + 3, l + 3, l + 3, l + 3,
                   l + 3],
                  [l, l - 1, l, l + 1, l - 2, l - 1, l, l + 1, l + 2, l - 3, l - 2, l - 1, l, l + 1, l + 2, l + 3],
                  [l, l - 1, l - 1, l - 1, l - 2, l - 2, l - 2, l - 2, l - 2, l - 3, l - 3, l - 3, l - 3, l - 3, l - 3,
                   l - 3]],

            "h": [[h, h + 1, h + 1, h + 1, h + 2, h + 2, h + 2, h + 2, h + 2, h + 3, h + 3, h + 3, h + 3, h + 3, h + 3,
                   h + 3],
                  [h, h + 1, h, h - 1, h + 2, h + 1, h, h - 1, h - 2, h + 3, h + 2, h + 1, h, h - 1, h - 2, h - 3],
                  [h, h - 1, h - 1, h - 1, h - 2, h - 2, h - 2, h - 2, h - 2, h - 3, h - 3, h - 3, h - 3, h - 3, h - 3,
                   h - 3],
                  [h, h - 1, h, h + 1, h - 2, h - 1, h, h + 1, h + 2, h - 3, h - 2, h - 1, h, h + 1, h + 2,
                   h + 3]]}  # up, right, down, left

    if d in [0, 1, 2, 3]:
        ls = np.array(viewsheds["l"][d])
        hs = np.array(viewsheds["h"][d])
    else:
        ls = np.array([item for sublist in viewsheds["l"] for item in sublist])
        hs = np.array([item for sublist in viewsheds["h"] for item in sublist])

    non_zero_idx = []
    noise_var = []
    count = 0
    for ii in range(0,len(ls)):
        if(ls[ii]<n1 and ls[ii]>=0 and hs[ii]<n2 and hs[ii]>=0):
            pos = int(hs[ii]*n1+ls[ii])
            non_zero_idx.append(pos)
            # noise_var.append(noise_vec[max(abs(ls[ii] - l), abs(hs[ii] - h))])
            noise_var.append(noise_vec[ii % len(noise_vec)])
            count = count+1
    x = np.zeros((count, n))
    for jj in range(count):
        x[jj,non_zero_idx[jj]] = 1

    return x,np.array(non_zero_idx),np.array(noise_var)

if __name__ == '__main__':
    rospy.init_node('simulate_threats')
    rospy.loginfo("SIMULATING THREATS FOR ZONE_RECON REPO")
    
    # Load parameters
    robot_name = rospy.get_param("~robot_name")
    cell_size = rospy.get_param("~cell_size")
    
    costmap = GetMessage(topic=rospy.get_param("~costmap_update_topic"), msg_type=OccupancyGrid, threading_lock=threading_lock)
    
    while not costmap.msg and rospy.get_param("~robot_type") == "ugv":
        rospy.sleep(0.5)
        rospy.loginfo_throttle(10, "Waiting for costmap info")
    
    if rospy.get_param("~robot_type") == "ugv":
        n1 = int(np.ceil(rospy.get_param("~costmap_cell_size") * rospy.get_param("~map_height") / (
            cell_size)))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
        n2 = int(np.ceil(rospy.get_param("~costmap_cell_size") * rospy.get_param("~map_width") / (
            cell_size)))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
        n = n1*n2
    else:
        exit("Threat faking for UAVs isn't implemented yet.")
    
    print("SIMULATE THREATS Height = %d, Width = %d"%(n1, n2))
    print("Number of cells are %d"%(n1*n2))
    
    ## Publish search polygon
    searchpolygon_publisher = SendMessage(topic = rospy.get_param("~search_polygon_topic"), msg_type=SearchPolygon, latched=True)
    while (searchpolygon_publisher.pub.get_num_connections() < 1):
        rospy.sleep(0.5)

    searchpolygon_publisher.msg.header.seq = 0
    searchpolygon_publisher.msg.header.stamp = rospy.Time().now()

    if rospy.get_param("~location") == "gascola":

        searchpolygon_publisher.msg.header.frame_id = "earth"

        searchpolygon_publisher.msg.polygon.points = [
            Point(x= 602668, y= 4479527, z= 0.0),
            Point(x= 602732, y= 4479363, z= 0.0),
            Point(x= 602635, y= 4479115, z= 0.0),
            Point(x= 602400, y= 4479215, z= 0.0),
            Point(x= 602603, y= 4479552, z= 0.0),
            Point(x= 602668, y= 4479527, z= 0.0)
        ]
    elif rospy.get_param("~location") == "ntc":
        searchpolygon_publisher.msg.header.frame_id = rospy.get_param("~robot_name") + \
                                                      rospy.get_param("~map_frame")

        searchpolygon_publisher.msg.polygon.points = [
            Point(x=2227.35888672, y=1431.94335938, z=0.0),
            Point(x=1714.18566895, y=10.3701705933, z=0.0),
            Point(x=133.517242432, y=604.304077148, z=0.0),
            Point(x=557.852172852, y=1820.10888672, z=0.0),
            Point(x=1047.26928711, y=1825.31591797, z=0.0),
            Point(x=2226.55615234, y=1440.00720215, z=0.0)
        ]

    # searchpolygon_publisher.msg.zone_id = 0
    searchpolygon_publisher.msg.robot_id = "lhex2"

    t = time.time()
    while time.time() < t+1:
        searchpolygon_publisher.publish()
        rospy.sleep(0.1)
        
    ##
    ## Fake Perception
    ##

    # Convert search_polygon to map frame and identify grid cells within search polygon
    points_map = []
    for point in searchpolygon_publisher.msg.polygon.points:
        tmp_pose = Pose()
        tmp_pose.position = point
        pose_map = None
        while not pose_map and not rospy.is_shutdown():
            pose_map = transform_pose(tmp_pose, searchpolygon_publisher.msg.header.frame_id,
                                        rospy.get_param("~robot_name") + rospy.get_param("~map_frame"))
        points_map.append(pose_map.position)

    searchpolygon_msg = copy.deepcopy(searchpolygon_publisher.msg)
    
    searchpolygon_msg.polygon.points = points_map
    searchpolygon_msg.header.frame_id = rospy.get_param("~robot_name") + rospy.get_param("~map_frame")

    grid_cell_size = rospy.get_param("~cell_size")
    inpolygon_grid_locations = []
    working_dir = "/home/" + str(os.environ['USER']) + "/src/zone_recon/"
    visibility_prior = image.imread(working_dir + "assets/" + \
                                         rospy.get_param("~location") + "/visibility_cost_" + \
                                         rospy.get_param("~map_type") + ".png")
    visibility_prior = rgb2gray(visibility_prior)
    print("Visibility prior dimensions", visibility_prior.shape)
    visibility_prior /= np.max(visibility_prior)
    probability_prior = compute_probability_prior(visibility_prior, costmap, n1, n2)

    probs = []
    for h in range(n2):
        for l in range(n1):
            x_map, y_map = grid2map((l,h), grid_cell_size)
            candidate_pose = Pose()
            candidate_pose.position.x = x_map 
            candidate_pose.position.y = y_map
            if is_within_search_region(candidate_pose.position, searchpolygon_msg):
                inpolygon_grid_locations.append((l,h))
                probs.append(probability_prior[l, h])
    probs = np.array(probs)/np.sum(probs)
    # Sample ground-truth object locations inside search polygon
    k = rospy.get_param("~num_targets")
    
    trl = rospy.get_param("~random_seed")
    print("SIMULATE THREATS RANDOM SEED RECEIVED IS: ", trl, "Numebr in poly grid locs: ", len(inpolygon_grid_locations))

    rng = np.random.RandomState(trl)
    print("\n\n\nSIMULATE THREATS INpolygridlocations: ", len(inpolygon_grid_locations))
    if rospy.get_param("~threat_sampling") == "stealth":
        print("WEEE USING THE STEALTH MODE \n\n\n")
        idx = rng.choice(len(inpolygon_grid_locations), k, replace=False, p=probs)
    else:
        idx = rng.choice(len(inpolygon_grid_locations), k, replace=False)

    mu = 1
    beta = np.zeros((n,1))
    print("Sampled threats at:")
    threat_locations_ground_truth = []
    for i in idx:
        beta[inpolygon_grid_locations[i][1]*n1+inpolygon_grid_locations[i][0], 0] = mu
        print("%d,%d"%(inpolygon_grid_locations[i][0], inpolygon_grid_locations[i][1]))
        threat_locations_ground_truth.append((inpolygon_grid_locations[i][0], inpolygon_grid_locations[i][1]))

    sigma2 = 0.005
    noise_vec = np.tile(np.array([sigma2, 4 * sigma2]), 4)
    print("THREAT FAKING FOR FOR ROBOT: ", robot_name, "with threat breakdown set to: ", rospy.get_param("~threat_breakdown"))
    all_topics = rospy.get_published_topics()
    # print("All topics: ", all_topics)
    current_ns = rospy.get_namespace()
    local_pose_topic = rospy.get_param("~local_pose_topic")
    all_ns = [topic[0].replace(local_pose_topic, "") for topic in all_topics if (local_pose_topic in topic[0])]
    print("Current namespace: ", [current_ns])
    # print("All namespaces: ", all_ns)
    track_topic = rospy.get_param("~track_topic")
    print("track topic: ", track_topic)

    # Suscribe to local_pose_topic and publish observations to ALL track_topics
    # if threat breakdown and current namespace not in
    #            NS not in topic|  F    T
    #   threat_breakdown        |
    #                       F       1    1
    #                       T       1    0
    # ^ desired behaviour, hence just take  !(TB and NS not in topic)
    # ~((rospy.get_param("~threat_breakdown")) and (current_ns not in topic[0])) and
    # boolean_check = [not (rospy.get_param("~threat_breakdown") and (current_ns != ns)) for ns in all_ns]
    # print("boolean check, one tru one false: ", boolean_check)
    track_topics = [ns+track_topic for ns in all_ns if not (rospy.get_param("~threat_breakdown") and (current_ns != ns))]
    print("track topics to publish to ", track_topics, "from ", robot_name)

    track_publishers = [SendMessage(topic, TrackDataVector) for topic in track_topics]
    print("publishers: ", track_publishers)

    last_pos = [-1,-1, -1]

    def simulate_tracks(pose_msg):
        if "start" in pose_msg.header.frame_id:
            pose_map = start2map(pose_msg)
        else:
            pose_map = pose_msg.pose.pose
        h, l = map2grid(pose_map.position, rospy.get_param("~cell_size"))
        quaternion = pose_map.orientation
        d = quaternion_to_facing_direction(quaternion)

        if [l, h, d] == last_pos:
            return
        last_pos[0] = l
        last_pos[1] = h
        last_pos[2] = d

        x, non_zero_idx, noise_var = create_directional_sensor(noise_vec, n1, n2, l, h, d)
        y = np.matmul(x, beta)

        # add noise
        epsilon = np.abs(np.multiply(rng.randn(x.shape[0],1),np.reshape(np.sqrt(noise_var),(-1,1))))
        epsilon[y > 0] = -epsilon[y > 0]

        y_noisy = np.clip(y + epsilon, 0, mu)

        # only publish if object is present
        if np.sum(y_noisy.squeeze() > 0.5):
            msg = TrackDataVector()
            for i in np.nonzero(y_noisy.squeeze() > 0.5)[0]:
                track = TrackData()
                track.track_id = int(non_zero_idx[i])

                ## convert grid -> map -> earth -> swap x and y idx
                l = non_zero_idx[i]%n1
                h = non_zero_idx[i]//n1
                print("Simulating threat at grid loc %d,%d,%d." % (l, h, d))
                # print("Simulating threat at grid loc %d,%d."%(h,l))

                map_x, map_y = grid2map((l, h), grid_cell_size)
                map_pose = PoseStamped()
                map_pose.pose.position.x = map_x
                map_pose.pose.position.y = map_y
                map_pose.pose.position.z = 0
                map_pose.header.frame_id = robot_name + rospy.get_param("~map_frame")

                tf_buffer = tf2_ros.Buffer()
                listener = tf2_ros.TransformListener(tf_buffer)

                track.header.frame_id = robot_name + rospy.get_param("~map_frame")
                track.position.x = map_pose.pose.position.x
                track.position.y = map_pose.pose.position.y

                t = rospy.get_rostime()
                track.latest_observation_timestamp = int(1000*rospy.Time.from_sec(time.time()).to_sec())
                track.weighted_confidence = y_noisy[i]
                track.covariance = [100*noise_var[i], 0, 0, 0, 100*noise_var[i], 0, 0, 0, 100*noise_var[i]]
                track.object_class = "simulated object"

                msg.data.append(track)
            
            [p.pub.publish(msg) for p in track_publishers]
        return
        
    pose_suscriber = GetMessage(topic=rospy.get_param("~local_pose_topic"), msg_type=Odometry, callback_modifier=simulate_tracks, threading_lock=threading_lock)

    try:
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
    except rospy.ROSInterruptException:
        exit("Exiting threat simulation")








