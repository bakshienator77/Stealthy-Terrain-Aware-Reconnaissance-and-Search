#!/usr/bin/env python


#

from __future__ import print_function

import rospy
import sys
import uuid
import tf2_ros
import threading
import copy
import time
import os

# Brings in the SimpleActionClient
import actionlib
import numpy as np

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped
from map_msgs.msg import OccupancyGridUpdate
from anon_msgs.msg import AutonomyActionGoal, AutonomyGoal, AutonomyAction
from anon_ros_msgs.msg import SearchPolygon, RobotObject
from zone_recon_msgs.msg import TrackDataVector
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray
from waypoint_planner.active_search import NATS
from waypoint_planner.waypoint_planner_utils import GetMessage, SendMessage, list_of_points3d_to_tuple2d, transform_pose
from waypoint_planner.waypoint_planner_utils import is_within_search_region, get_nearest_vertex, grid2map, map2grid
from waypoint_planner.waypoint_planner_utils import init_searchpolygon_marker, start2grid, start2map, \
    costmap2slice, globworld2earth, earth2map, update_null_observation, remove_duplicates, get_point_on_polygon, \
    get_dummy_goal, quaternion_to_facing_direction
from std_srvs.srv import SetBool
import tf2_geometry_msgs  # To be able to transform between frames

from matplotlib import pyplot as plt
import cProfile
import pstats
# import ipdb

is_pause = False
threading_lock = threading.Lock()
waypoint_pose_stamped_publisher = SendMessage(topic="waypoint_viz", msg_type=PoseStamped)

class LocalPoseSubscriber():
    '''
        This subscriber object maintains the latest local pose odometry message, it also imputes negative observations
        on the basis of the location and heading of the robot

        track_subscriber: object of TrackSubscriber class to be used to prevent adding
        a negative observation when a positive one is available [currently unused]
        nats_obj: custom defined object that runs the active search algorithm and maintains observation list
        topic: topic at which local pose is published
        msg_type: type of message for local pose
        verbose: boolean option to print message data upon each call back
        callback_modifier: Optional function that is run on the message data every time callback is triggered
    '''
    def __init__(self, track_subscriber, nats_obj, topic, msg_type, verbose=False, callback_modifier=None): #topic name to be in yaml file
        self.msg = None
        self.sub = rospy.Subscriber(topic, msg_type, self.callback, queue_size=10000)
        self.lock = threading_lock
        self.verbose = verbose
        self.callback_modifier = callback_modifier
        self.track_subscriber = track_subscriber
        self.grid_coord = None
        self.nats_obj = nats_obj
        self.previous_pose_start = None

    def callback(self, data):
        # When current pose changes coordinates in grid frame take new reading
        # which is imputed as a zero reading unless track reading is live
        with self.lock:
            # print("Local Pose Subscriber Callback started")
            t_0 = rospy.get_time()
            if self.verbose:
                print(data)

            self.msg = data
            self.nats_obj.current_pose = data

            if self.callback_modifier is not None:
                self.callback_modifier(data)

            robot_type = rospy.get_param("~robot_type") # I am who I am told I am
            current_pose = data

            if self.previous_pose_start is None:
                self.previous_pose_start = current_pose
            else:
                # check if robot has moved since last observation
                old_point = self.previous_pose_start.pose.pose.position
                current_point = current_pose.pose.pose.position
                distance = (old_point.x - current_point.x)**2 + (old_point.y - current_point.y)**2+ (old_point.z - current_point.z)**2
                old_d = quaternion_to_facing_direction(self.previous_pose_start.pose.pose.orientation)
                new_d = quaternion_to_facing_direction(current_pose.pose.pose.orientation)
                if distance > 3 or old_d != new_d:
                    self.previous_pose_start = current_pose
                else:
                    # print("\n\n\nIAM RETURNING IN THE LOCAL POSE SUBSCRIBER\n\n\n")
                    # print(self.previous_pose_start)
                    # print(current_pose)
                    # print("Time taken on callback: ", rospy.get_time() - t_0)
                    return

            # TODO: Only when no track seen right now??? [Absence of this doesn't seem to adversely impact performance]
            if "start" in current_pose.header.frame_id:
                pose_map = start2map(current_pose)
            else:
                pose_map = current_pose.pose.pose
            self.grid_coord = update_null_observation(pose_map, self.nats_obj, robot_type, self.grid_coord)
            # print("Local Pose Subscriber completed with grid_coord update")
            # print("Time taken on callback: ", rospy.get_time() -t_0)

    def get_copy(self):
        with self.lock:
            return self.msg


class TrackSubscriber():
    '''
        This subscriber object keeps track of incoming track information, adds it to the nats_obj observation
        list and publishes a visualisation marker for RVIZ

        nats_obj: custom defined object that runs the active search algorithm and maintains observation list
        topic: topic at which track info is published
        msg_type: type of message for track
        verbose: boolean option to print message data upon each call back
        callback_modifier: Optional function that is run on the message data every time callback is triggered
    '''

    def __init__(self, nats_obj, topic, msg_type, verbose=False, callback_modifier=None):
        self.msg = None
        self.sub = rospy.Subscriber(topic, msg_type, self.callback, queue_size=10000)  # , queue_size=1)
        self.lock = threading_lock
        self.verbose = verbose
        self.callback_modifier = callback_modifier
        self.previous_tracks = dict()
        # self.points_dict = points_dict
        self.timer = rospy.rostime.get_rostime()
        self.nats_obj = nats_obj
        self.marker_publisher = SendMessage(rospy.get_param("~detected_threats_topic"), MarkerArray)

    def populate_marker_msg(self, markerarray_msg, point, track, uncertainty):
        '''
        markerarray_msg: MarkerArray type message
        point: location of track in map frame
        track: TrackData type message object containing track information from perception
        uncertainty: Circle radius indicating uncertainty, pass False if the desired behaviour is to use the
                    true ellipsoid shape of location uncertainty in visualisation

        returns: nothing, directly modifies the markerarray_msg object
        '''
        preexisting = False
        for marker in markerarray_msg.markers:
            if track.track_id == marker.id:
                #simply update track
                preexisting = True
                self.fill_pose_info(marker, point, track, uncertainty)
                break
        if not preexisting:
            #brand new track id
            marker = Marker()
            marker.header.frame_id = rospy.get_param("~robot_name") + rospy.get_param("~map_frame")
            marker.id = track.track_id
            marker.action = Marker.ADD
            # publisher.msg.pose.orientation.w = 1.0
            marker.type = Marker.SPHERE
            marker.color.a = 1.0
            self.fill_pose_info(marker, point, track, uncertainty)
            # print(marker.pose)
            markerarray_msg.markers.append(marker)

    def fill_pose_info(self, marker, point, track, circle=False):
        '''
        marker: marker message object
        point: location of track in map frame
        track: TrackData type message object containing track information from perception
        circle: Circle radius indicating uncertainty, pass False if the desired behaviour is to use the
                    true ellipsoid shape of location uncertainty in visualisation

        returns: nothing, directly modifies the marker object

        '''
        marker.header.stamp = rospy.Time.now()
        if circle:
            # ignoring ellipsoid properties of the location uncertainty and instead approximating as a sphere
            marker.scale.x = circle
            marker.scale.y = circle
            marker.scale.z = circle
        else:
            # extracting eigen values and vectors from covariance matrix
            eigvals, eigvecs = np.linalg.eig(np.array(track.covariance).reshape(3, 3))
            # x, y and z are the 3 eigen vectors, will rotate it to align it below
            marker.scale.x = eigvals[0]
            marker.scale.y = eigvals[1]
            marker.scale.z = eigvals[2]
            theta = np.arccos((np.trace(eigvecs) - 1) / 2) # extracting angle
            direction = np.linalg.eig(eigvecs)[1][:, 0] # extracting axis
            # converting from angle axis to quaternion representation
            marker.pose.orientation.w = np.cos(theta / 2)
            marker.pose.orientation.x = np.sin(theta / 2) * direction[0]
            marker.pose.orientation.y = np.sin(theta / 2) * direction[1]
            marker.pose.orientation.z = np.sin(theta / 2) * direction[2]
        # Brightness of the green indicates the confidence in the track
        if track.object_class == "person":
            marker.color.r = track.weighted_confidence
        elif track.object_class == "pickuptruck":
            marker.color.g = track.weighted_confidence
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = track.weighted_confidence

        marker.pose.position = point

    def get_copy(self):
        with self.lock:
            #return copy.deepcopy(self.msg)
            return self.msg

    def callback(self, msg):
        with self.lock:
            if self.verbose:
                print(self.msg)
            if self.callback_modifier is not None:
                self.callback_modifier(msg)
            self.msg = msg
            for track in msg.data:
                # for each track
                # print(track)
                if not self.previous_tracks.get(track.track_id):
                    # new track
                    self.previous_tracks[track.track_id] = track
                    # update observations
                elif track.latest_observation_timestamp > self.previous_tracks[track.track_id].latest_observation_timestamp:
                    # have seen this object before but have a newer observation
                    self.previous_tracks[track.track_id] = track
                    # update observations
                else:
                    continue
                # if track.latest_robot == rospy.get_param("~robot_name"):
                    # I am the robot
                x = np.zeros((1,self.nats_obj.n))
                uncertainty = np.linalg.norm(np.array(track.covariance)) # frobenious norm approximate for depth aware noise
                rospy.loginfo("Location Uncertainty: %f" % uncertainty)
                uncertainty = min(0.5, uncertainty / (1000 * track.weighted_confidence))
                rospy.loginfo("Heuristic Detection Uncertainty: %f" % uncertainty)
                Y = np.array([[track.weighted_confidence, uncertainty]])
                if "map" in track.header.frame_id:
                    map_pose_position = track.position
                else:
                    map_pose = earth2map(globworld2earth(track))
                    map_pose_position = map_pose.point
                grid_coords = map2grid(map_pose_position, rospy.get_param("~cell_size"))
                h, l = grid_coords
                print("Grid coords of track are: ", grid_coords)
                print("Map coordinates of track are: ", map_pose_position)
                print("Object class of track is: ", track.object_class)
                # False indicates that the true ellipsoid shape should be used for visualisation
                #
                self.populate_marker_msg(self.marker_publisher.msg, map_pose_position, track, False)
                # print("shape of x is: ", x.shape)
                x[0, h*self.nats_obj.n1 + l] = 1  # one-hot with cell in which track currently is
                # self.nats_obj.points_dict['Y'] = np.append(self.nats_obj.points_dict['Y'], Y, axis=0)
                # TODO: Is par even used?
                # self.nats_obj.points_dict['par'] = [0, np.array([grid_coords[0]*grid_coords[1]])]
                # self.nats_obj.points_dict['X'] = np.append(self.nats_obj.points_dict['X'], x, axis=0)
                # print("We are updating: ", x, Y)
                if not np.any(self.nats_obj.points_dict['X']):
                    # first time
                    self.nats_obj.points_dict['X'] = x
                    self.nats_obj.points_dict['Y'] = Y
                    self.nats_obj.points_dict['par'] = [0, np.array([h*self.nats_obj.n1+l])]
                else:
                    # some appending magic
                    self.nats_obj.points_dict['X'] = np.append(self.nats_obj.points_dict['X'], x, axis=0)
                    self.nats_obj.points_dict['Y'] = np.append(self.nats_obj.points_dict['Y'], Y, axis=0)
                    self.nats_obj.points_dict['par'] = [0, np.array([h*self.nats_obj.n1+l])]
            self.marker_publisher.publish()

# <<<<<<< HEAD
            # TODO: Look into setting an epoch for this computation
            # self.nats_obj.compute_and_save_log()
# =======
#             self.nats_obj.publish_detected_threats()
# >>>>>>> tejus/evaluate_multirobot
            # IT DOESNT MATTER WHO THE VIEWING ROBOT IS BECAUSE THE X IS GOING TO BE A ONE HOT VECTOR THE
            # Y IS GOIN TO BE NON ZERO ONLY WITH THE CONFIDENCE AND THE VARIANCE US GOING TO BE FROB NORM
            # OF LOC UNCERTAINTY
            # When new track info received update observations
            # Is it new track information?
            # how much time since latest track update? Useful when need to impute zero obs
            # If yes, pull corresponding latest_robot's pose at the timestamp
            # construct the visibility matrix on the basis of it
            # transform from earth to map after inverting x and y
            # transform from map to grid, from grid to beta position
            # compute the uncertainties using frobenius norm of covariance (location uncertainty)
            # append X, Y[obs, var] to points_dict
            return 0

class CrossPoseSubscriber():
    '''
        This subscriber object keeps track of incoming robot pose/object information, and similar to
        localposesubscriber imputes negative observations and adds it to the nats_obj observations list.
        It also visualises the other robots on RVIZ

        track_subscriber: object of TrackSubscriber class to be used to prevent adding
        nats_obj: custom defined object that runs the active search algorithm and maintains observation list
        topic: topic at which track info is published
        msg_type: type of message for track
        verbose: boolean option to print message data upon each call back
        callback_modifier: Optional function that is run on the message data every time callback is triggered
    '''
    # TODO: add visualisation as an optional param
    def __init__(self, track_subscriber, nats_obj, topic, msg_type, verbose=False, callback_modifier=None):
        self.msg = None
        self.sub = rospy.Subscriber(topic, msg_type, self.callback , queue_size=10000)
        self.lock = threading_lock
        self.verbose = verbose
        self.callback_modifier = callback_modifier
        self.previous_tracks = dict()
        self.track_subscriber = track_subscriber
        # self.points_dict = points_dict
        self.timer = rospy.rostime.get_rostime()
        self.nats_obj = nats_obj
        self.grid_coord = dict()
        # TODO: parameterise the following topic
        self.marker_publisher = init_searchpolygon_marker(SendMessage("seen_robots_path", Marker))
        self.marker_publisher.msg.type = Marker.LINE_STRIP
        # self.marker_publisher.msg.colors = []
        # self.marker_publisher.msg.scale.z = 5.0
        # self.marker_publisher.msg.scale.x = 5.0
        self.marker_publisher.msg.points = []
        self.marker_publisher.msg.header.frame_id = rospy.get_param("~robot_name") + rospy.get_param("~map_frame")
        # TODO: parameterise the following topic
        # visualisation publisher for the
        self.pose_publisher = SendMessage("pose_seen_robot", PoseStamped)

    def get_copy(self):
        with self.lock:
            #return copy.deepcopy(self.msg)
            return self.msg

    def callback(self, msg):
        # When current pose changes coordinates in grid frame take new reading
        # which is imputed as a zero reading [unless track reading is live] (distinction not implemented yet)
        with self.lock:
            # print("CrossPose Subscriber Callback started")
            t_0 = rospy.get_time()
            self.msg = msg
            if self.verbose:
                print(self.msg)
            if self.callback_modifier is not None:
                self.callback_modifier(msg)
            robot_name = rospy.get_param("~robot_name")
            # message of type RobotObjectSet
            #robot_object = copy.deepcopy(msg)
            robot_object = msg
            robot_type = "uas" if "hex" in robot_object.robot_id else "ugv"
#            print("Cross pose (Should be changing): ", robot_poses)
            # TODO: Only when no track seen right now???
            # print("I AM IN THE receiving portion OF THE CROSSTRACK SUBSCRIBER")

            if robot_name != robot_object.robot_id:
                # print("I AM IN THE PUBLISHING PROTION OF THE CROSSTRACK SUBSCRIBER", robot_name)
                if not self.grid_coord.get(robot_object.robot_id):
                    # First time
                    self.grid_coord[robot_object.robot_id] = None
                
                pose_map = None 
                while not pose_map and not rospy.is_shutdown():
                    if robot_object.header.frame_id == "global":
                        pose_earth = self.global2earth(robot_object)
                        pose_map = transform_pose(pose_earth.pose, "earth", robot_name+rospy.get_param("~map_frame"))
                    else:
                        pose_map = transform_pose(robot_object.pose, robot_object.header.frame_id, robot_name+rospy.get_param("~map_frame"))
                    
                    # rospy.sleep(0.02)
                # Viz for non-self robot, yet to be tested with multiple other robots
                # but my intuition is it will still work although the pose marker may blink
                # TODO: consider usefulness of path marker publisher
                # self.marker_publisher.msg.points.extend([pose_map.position])
                # self.marker_publisher.publish()
                self.pose_publisher.msg.pose = pose_map
                self.pose_publisher.msg.header = robot_object.header
                self.pose_publisher.msg.header.frame_id = robot_name+rospy.get_param("~map_frame")
                self.pose_publisher.msg.header.stamp = rospy.rostime.get_rostime()
                self.pose_publisher.publish()
                self.grid_coord[robot_object.robot_id] = update_null_observation(pose_map, self.nats_obj, robot_type, self.grid_coord[robot_object.robot_id], robot_object.robot_id)
            # print("CrossPose Subscriber completed with grid_coord update ", rospy.get_time() -t_0)


    def global2earth(self, current_pose):
        pose_earth = PoseStamped()
        pose_earth.header = current_pose.header
        pose_earth.header.frame_id = "earth"
        pose_earth.header.stamp = rospy.Time.now()  # for now, eventually mesh bridge will take care of this
        temp = [0.7071068, 0.7071608, 0, 0]
        original = [current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w]
        n = quaternion_multiply(temp, original)
        pose_earth.pose.orientation.x = n[0]
        pose_earth.pose.orientation.y = n[1]
        pose_earth.pose.orientation.z = n[2]
        pose_earth.pose.orientation.w = n[3]
        pose_earth.pose.position.x = current_pose.pose.position.y
        pose_earth.pose.position.y = current_pose.pose.position.x
        pose_earth.pose.position.z = - current_pose.pose.position.z
        return pose_earth

    # Detect a crosspose message
    # go through the list and for every robot that is not itself
    # Similar to local pose subscriber if the position/orientation has not changed from the last update, wait
    # If changed then:
    # constructs the FOV
    # TODO: how to not overwrite an observation?
    # append the observation to points dict



def get_next_waypoint(waypoint, nats_obj, first_time=False):
    '''
    The following happening in callback of track subscriber:
        New track that is unprocessed?
        If not assume yt = [0 0 0 0 0 0 ...] and append Xt and yt to X and y
    When this function is called then call Active Search:
        The search region over the entire map is decided by the search polygon
        Return Waypoint
    '''
    # TODO: nats_obj contains the observations for now, think of decoupling later
    # Running NATS
    pose_map = Pose()
    result = None
    if not is_within_search_region(start2map(nats_obj.current_pose).position,
                                   nats_obj.search_polygon.get_copy()) and \
            rospy.get_param("~robot_type") == "uas":
        rospy.loginfo("Robot not within search polygon")
        # if I am not currently in search zone
        if (first_time):
            rospy.loginfo("heading to nearest point on polygon")
            # OR this is first_time and I am a drone
            # get to the search region ASAP
            position_map = start2map(nats_obj.current_pose).position
            search_polygon_msg = nats_obj.search_polygon.get_copy()
            x, y = get_point_on_polygon(position_map, search_polygon_msg)
            pose_map.position.x = x
            pose_map.position.y = y
            nats_obj.current_waypoint = list(map2grid(pose_map.position, rospy.get_param("~cell_size")))
            nats_obj.current_waypoint.append(0)
            return as_goal_with_viz(pose_map)
    
        if not is_within_search_region(transform_pose(waypoint.path.poses[0].pose,
                                                      nats_obj.current_pose.header.frame_id,
                                                      nats_obj.search_polygon.get_copy().header.frame_id).position,
                                       nats_obj.search_polygon.get_copy()):# or (first_time and rospy.get_param("~robot_type") == "ugv"):
            #, and (waypoint is also not in current search zone or UGV's first time
            rospy.loginfo("Doing one step random waypoint into the search polygon")
            t0 = time.time()
            # profiler = cProfile.Profile()
            # profiler.enable()
            result = nats_obj.ActiveSearch("coverage")
            # profiler.disable()
            # stats = pstats.Stats(profiler).sort_stats('ncalls')
            # stats.print_stats()
            t1 = time.time()

    if result is None:
        t0 = time.time()
        # profiler = cProfile.Profile()
        # profiler.enable()
        result = nats_obj.ActiveSearch()
        # profiler.disable()
        # stats = pstats.Stats(profiler).sort_stats('ncalls')
        # stats.print_stats()
        t1 = time.time()
    rospy.loginfo("Time taken on this iteration of NATS: %f" % (t1 - t0))

    coords = grid2map(result['grid_pos'], cell_size=rospy.get_param("~cell_size"))
    # print(coords)

    # Transform to map coords
    pose_map.position.x = coords[1]
    pose_map.position.y = coords[0]
    quaternion = quaternion_from_euler(0, 0,result['grid_pos'][2]*1.57)
    # print("grid direction: ", result['grid_pos'][2], " corresponds to quaternion in map frame: ", quaternion)
    pose_map.orientation.x = quaternion[0]
    pose_map.orientation.y = quaternion[1]
    pose_map.orientation.z = quaternion[2]
    pose_map.orientation.w = quaternion[3]

    goal = as_goal_with_viz(pose_map)
    return goal

def as_goal_with_viz(pose_map):
    # Visualise the goal
    pose_stamped_map = PoseStamped()
    pose_stamped_map.pose = pose_map
    pose_stamped_map.header.frame_id = robot_name + rospy.get_param("~map_frame")
    pose_stamped_map.header.stamp = rospy.Time.now()
    waypoint_pose_stamped_publisher.msg = pose_stamped_map
    waypoint_pose_stamped_publisher.publish()
    # Transform to start frame for goal specification to action server
    pose_goal = None
    while not pose_goal and not rospy.is_shutdown():
        rospy.sleep(0.1)
        pose_goal = transform_pose(pose_map, robot_name + rospy.get_param("~map_frame"),
                                   robot_name + rospy.get_param("~goal_output_frame"))
    # print("Goal coords in map frame: ", pose_map)
    # print("Goal coords in start frame: ", pose_start)
    # Set and return goal
    goal = get_dummy_goal()
    goal.path.poses[0].pose = pose_goal
    return goal


def waypoint_action_client(client, prev_waypoint, nats_obj=None, first_time=False):
    '''
    Skeleton waypoint action client function that is called in a loop from main
    '''

    # Waits until the action server has started up and started
    # listening for goals.
    # Creates a goal to send to the action server.
    if not rospy.get_param("~simultaneous_compute") and not first_time:
        rospy.loginfo("waiting on result ...")
        client.wait_for_result()
    goal = get_next_waypoint(prev_waypoint, nats_obj, first_time)

    t0 = time.time()
    if first_time:
        rospy.loginfo("Waiting for action server to start ...")
        client.wait_for_server()
    elif rospy.get_param("~simultaneous_compute"):
        rospy.loginfo("waiting on result ...")
        client.wait_for_result()
    t1 = time.time()
    rospy.loginfo("Extra time available to plan: %f" % (t1 - t0))

    # Sends the goal to the action server.
    rospy.loginfo("sending goal...")
    if not is_pause:
        client.send_goal(goal)

    # Prints out the result of executing the action
    return client.get_result(), goal

def handle_pause(req):
    global is_pause
    is_pause = req.data
    if is_pause:
        return (True, "Pausing")
    else:
        return (True, "Restarting")

initialize_request = False 

def handle_initialize_request(req):
    global initialize_request
    initialize_request = req.data

    return (True, "Reinitializing")



if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('waypoint_planner')
        rospy.loginfo("RUNNING WAYPOINT_PLANNER FROM ZONE_RECON REPO")

        # initialize service to pause zone_recon
        pause_service = rospy.Service('pause', SetBool, handle_pause)

        # initialize service to initialize zone_recon
        # this resets NATS' observation buffer
        initialize_service = rospy.Service('initialize_zone_recon', SetBool, handle_initialize_request)

        # initialize suscriber to search_polygon and costmap; publisher for search_polygon_viz
        robot_name = rospy.get_param("~robot_name")
        costmap = GetMessage(topic=rospy.get_param("~costmap_update_topic"), msg_type=OccupancyGrid)

        sp_publisher = init_searchpolygon_marker(SendMessage(topic=rospy.get_param("~search_polygon_viz_topic"), msg_type=Marker))
        
        print("EXPECTED NAMESAPCE: ", rospy.get_namespace())
        def search_polygon_to_map_and_viz(search_polygon_subscriber_msg):
            # takes search_polygon msg, converts it to map frame and publishes to search_polygon_viz_topic
            print("Received search polygon")
            if search_polygon_subscriber_msg is not None:
                points_map = []
                for point in search_polygon_subscriber_msg.polygon.points:
                    temp_pose = Pose()
                    temp_pose.position = point
                    pose_map = None
                    while not pose_map and not rospy.is_shutdown():
                        pose_map = transform_pose(temp_pose, search_polygon_subscriber_msg.header.frame_id,
                                                  rospy.get_param("~robot_name") + rospy.get_param("~map_frame"))
                    points_map.append(pose_map.position)
                search_polygon_subscriber_msg.polygon.points = points_map
                search_polygon_subscriber_msg.header.frame_id = rospy.get_param("~robot_name") + rospy.get_param("~map_frame")
                sp_publisher.msg.points = search_polygon_subscriber_msg.polygon.points
                sp_publisher.msg.points.append(search_polygon_subscriber_msg.polygon.points[0])
                sp_publisher.msg.header.frame_id = search_polygon_subscriber_msg.header.frame_id
                sp_publisher.publish()

                if search_polygon_subscriber_msg.zone_clearing:
                    # overwrite params from config file only when ATAK tells you something of value
                    rospy.set_param("~zone_clearing", True)
                    rospy.set_param("~distance_cost_weightage", 0.3)
                if search_polygon_subscriber_msg.stealth:
                    rospy.set_param("~visibility_cost_weightage", 0.03 * (search_polygon_subscriber_msg.stealth/100.0))

        
        search_polygon = GetMessage(topic=rospy.get_param("~search_polygon_topic"), msg_type=SearchPolygon, callback_modifier=search_polygon_to_map_and_viz)

        # NATS

        lmbd = 10  # Laplace hyper parameter lmbd = sqrt(eta) where eta is introduced in the paper
        sigma2 = 0.005  # noise variance on observations
        trl = sum([ord(c) for c in robot_name]) # responsible for generating a random_state

        # Omni view assumption
        # noise_vec = np.tile(np.array([sigma2, 4 * sigma2]), 4)
        # Old Ramina assumption
        # noise_vec = np.append(np.append(np.array(1*[sigma2,sigma2, sigma2]),
        #                                  np.repeat(4*sigma2,5)),np.repeat(9*sigma2,7))
        # Directional view assumption
        # noise_vec = np.append(np.array([sigma2]), np.array([4*sigma2]))
        noise_vec = dict()
        noise_vec["uas"] = np.array([8*sigma2])
        # Omni view assumption
        if rospy.get_param("~fov_setting") == "conservative":
            noise_vec["ugv"] = np.tile(np.array([sigma2, 4 * sigma2]), 4)
        elif rospy.get_param("~fov_setting") == "viewshed":
            # noise_vec = np.append(np.append(np.array(1*[sigma2,sigma2, sigma2]),
            #                                  np.repeat(4*sigma2,5)),np.repeat(9*sigma2,7))
            noise_vec["ugv"] = np.array([sigma2/5, sigma2, 4 * sigma2, 9 * sigma2])
        else:
            rospy.loginfo("Who do you, Who do you, Who do you, WHo do you think you are? ha ha ha wrong fov_setting in nats_param.yaml file")

        cell_size = rospy.get_param("~cell_size")
        
        while not costmap.msg and rospy.get_param("~robot_type") == "ugv" and not rospy.is_shutdown():
            rospy.sleep(0.5)
            rospy.loginfo_throttle(10, "Waiting for costmap info")

        if rospy.get_param("~robot_type") == "ugv":
            n1 = int(np.ceil(costmap.msg.info.height * costmap.msg.info.resolution / cell_size)) # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
            n2 = int(np.ceil(costmap.msg.info.width * costmap.msg.info.resolution / cell_size)) # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
        if rospy.get_param("~robot_type") == "uas":
            n1 = int(np.ceil(rospy.get_param("~costmap_cell_size") * rospy.get_param("~map_height") / (cell_size)))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
            n2 = int(np.ceil(rospy.get_param("~costmap_cell_size") * rospy.get_param("~map_width") / (cell_size)))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
        
        print("Grid-size:%dx%d with each cell %dmx%dm"%(n1, n2, cell_size, cell_size))

        noise_aware_ts = NATS(beta=np.zeros(n1*n2),
                              n1=n1, 
                              mu=1.0,
                              noise_vec=noise_vec,
                              lmbd=lmbd,
                              EMitr=1,# try 10 to start: number of iterations for the Expectation-Maximization estimator # can also try ending using a tolerance for the sigma_beta or beta_hat
                              n_agents=1,
                              trl=trl,
                              search_polygon=search_polygon,
                              costmap=costmap)
        
        # suscribe to pose and tracks
        track_subscriber = TrackSubscriber(noise_aware_ts, topic=rospy.get_param("~track_topic"), msg_type=TrackDataVector)
        local_pose = LocalPoseSubscriber(track_subscriber, noise_aware_ts, topic=rospy.get_param("~local_pose_topic"), msg_type=Odometry)
        cross_pose = CrossPoseSubscriber(track_subscriber, noise_aware_ts, topic=rospy.get_param("~cross_pose_topic"), msg_type=RobotObject)

        search_mode = rospy.get_param("~search_mode")

        rospy.loginfo("Setting up client ...")
        client = actionlib.SimpleActionClient(rospy.get_param("~autonomy_server_topic"), AutonomyAction)
        rospy.loginfo("done")

        while not local_pose.msg:
            rospy.loginfo_throttle(5, "No local pose available, waiting...")
            rospy.sleep(1)
        if rospy.get_param("~zone_clearing"):
            noise_aware_ts.oldh, noise_aware_ts.oldl = start2grid(local_pose.msg, rospy.get_param("~cell_size"))

        first_time = True
        waypoint = get_dummy_goal()
        global initialize_request

        while not rospy.is_shutdown():
            if not is_pause and search_polygon.msg is not None:
                trl += 1
                result, waypoint = waypoint_action_client(client, waypoint, nats_obj=noise_aware_ts, first_time=first_time)
                first_time = False
            elif is_pause:
                rospy.loginfo_throttle(30, "waypoint planner currently paused")
            elif search_polygon.msg is None:
                rospy.loginfo_throttle(5, "Please specify a search region for the waypoint planner to start.")
                rospy.sleep(1)
            # posterior building will still happen regardless of being paused

            if initialize_request:
                # on re-initialization, empty observation buffer and create new logging director
                print("\n\n REINITIALIZING NATS \n\n")
                noise_aware_ts.points_dict = {'X': np.zeros_like(noise_aware_ts.points_dict["X"][:2,:]),
                                              'Y': np.zeros_like(noise_aware_ts.points_dict["Y"][:2,:]),
                                              'par': noise_aware_ts.points_dict['par'] }
                noise_aware_ts.robot_locs = []
                noise_aware_ts.robot_goals = []
                noise_aware_ts.last_loc = None

                local_pose.previous_pose_start = None
                track_subscriber.previous_tracks = dict()
                cross_pose.previous_tracks = dict()
                rospy.sleep(1)

                if rospy.get_param("~logging"):
                    noise_aware_ts.logging_path = rospy.get_param("~logging_path") + str(time.asctime( time.localtime(time.time()) )) + "/"
                    rospy.loginfo("Zone Recon Metrics Logging path: " + str(noise_aware_ts.logging_path))
                    if not os.path.exists(noise_aware_ts.logging_path):
                        os.makedirs(noise_aware_ts.logging_path)

                initialize_request = False
            # rospy.sleep(0.1)
            pose = local_pose.get_copy()


    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)

