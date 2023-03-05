

from __future__ import print_function

import rospy
import sys
import uuid
import tf2_ros

# Brings in the SimpleActionClient
import actionlib
import numpy as np
import threading
import copy

from nav_msgs.msg import Path, OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, PointStamped, PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from anon_msgs.msg import AutonomyActionGoal, AutonomyGoal, AutonomyAction
from anon_ros_msgs.msg import SearchPolygon, RobotObject
from zone_recon_msgs.msg import TrackDataVector
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from waypoint_planner.polygon_utils import is_inside_polygon
from std_srvs.srv import SetBool
import tf2_geometry_msgs  # To be able to transform between frames
import matplotlib
import time
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from PIL import Image

is_pause = False


class GetMessage():
    '''
    Template subscriber class whose msg object should always be the latest msg data

    topic: topic at which local pose is published
    msg_type: type of message for local pose
    verbose: boolean option to print message data upon each call back
    callback_modifier: Optional function that is run on the message data every time callback is triggered

    '''
    def __init__(self, topic, msg_type, verbose=False, callback_modifier=None, threading_lock=None): #topic name to be in yaml file
        self.msg = None
        self.sub = rospy.Subscriber(topic, msg_type, self.callback, queue_size=10000)
        if threading_lock is None:
            self.lock = threading.Lock()
        else:
            self.lock = threading_lock
        self.verbose = verbose
        self.callback_modifier = callback_modifier

    def callback(self, data):
        with self.lock:
            self.msg = data
            if self.verbose:
                print("Original callback: ", self.msg)
            if self.callback_modifier is not None:
                self.callback_modifier(data)

    def get_copy(self):
        with self.lock:
            return copy.deepcopy(self.msg)

class SendMessage():
    '''
    Template Publisher class, topic and msg_type need to be specified upon instantiation
    To use, update the msg attribute with the required data and call the publish attribute

    topic: topic to which information should be published
    msg_type: type of message

    '''

    def __init__(self, topic, msg_type, latched=False):
        self.msg = msg_type()
        self.pub = rospy.Publisher(topic, msg_type, queue_size=10000, latch=latched)

    def publish(self):
        self.pub.publish(self.msg)

def list_of_points3d_to_tuple2d(points):
    '''
    Generic ros messages often have 3d specifications, this helper function is more suited to working 2d
    TODO: May need to rethink when moving to UAVs [Not for Aug 2021 bec height is not in algo control]
    '''
    points2d = []
    for point in points:
        points2d.append((point.x, point.y))
    return points2d

def transform_pose(input_pose, from_frame, to_frame, timestamp=None):
    '''
    input_pose: is a Pose() message type that can be from geometry_msgs
    from_frame: is the string frame name  which the above is specified
    to_frame: is a string frame name to which the pose should be transformed
    timestamp (optional): If the time of the transform required is not the current time

    returns: Pose() type message in output frame
    '''
    if from_frame == to_frame:
        return input_pose
    if "map" in from_frame and "map" in to_frame:
        return input_pose

    # odom frame unreliable and hence we approximate as start frame
    if from_frame == (rospy.get_param("~robot_name") + rospy.get_param("~odom_frame")):
        # print("FROM FRAME IS: ", from_frame)
        from_frame = rospy.get_param("~robot_name") + rospy.get_param("~start_frame")
    if from_frame == (rospy.get_param("~robot_name") + "/odom"):
        from_frame = rospy.get_param("~robot_name") + "/start"

    if to_frame == (rospy.get_param("~robot_name") + rospy.get_param("~odom_frame")):
        # print("TO FRAME IS: ", to_frame)
        to_frame = rospy.get_param("~robot_name") + rospy.get_param("~start_frame")
    if to_frame == (rospy.get_param("~robot_name") + "/odom"):
        to_frame = rospy.get_param("~robot_name") + "/start"

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    # print(tf_buffer.registration.print_me())
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    if not timestamp:
        pose_stamped.header.stamp = rospy.Time.now()
    else:
        pose_stamped.header.stamp = timestamp
    # rospy.sleep(0.1)
    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        # print(tf_buffer.lookup)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame)#, rospy.Duration(10))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo("Houston we have a problem with the transform (%s->%s)..."%(from_frame, to_frame) + str(e))
        # rospy.loginfo("Pose in question: " + str(pose_stamped) )

def is_within_search_region(point_3d, search_polygon_msg):
    '''
    Handles error case that no search polygon corresponding to this robot has been specified
    '''
    if search_polygon_msg is None:
        rospy.loginfo("The is no valid search polygon specified.")
        return False
    else:
        return is_inside_polygon(list_of_points3d_to_tuple2d(search_polygon_msg.polygon.points),(point_3d.x, point_3d.y) )

def get_nearest_vertex(point_3d, search_polygon_msg, second_nearest=False):

    def dist(p1,p2):
        return np.sqrt((p1[0] -p2[0])**2 + (p1[1] -p2[1])**2)
    point_list = list_of_points3d_to_tuple2d(search_polygon_msg.polygon.points)
    mini = float("inf")
    loc = None
    loc_2 = None
    for i, point in enumerate(point_list):
        d = dist(point, (point_3d.x, point_3d.y))
        if d < mini:
            if loc is not None:
                loc_2 = loc
            loc = i
            mini= d
    if second_nearest:
        if loc_2 is None:
            loc_2 = loc+1
        return point_list[loc], point_list[loc_2]
    return point_list[loc]

def globworld2earth(trackdata):
    ptst = tf2_geometry_msgs.PointStamped()
    ptst.header = trackdata.header
    ptst.header.frame_id = "earth"
    ptst.point.z = trackdata.position.z
    ptst.point.y = trackdata.position.x
    ptst.point.x = trackdata.position.y

    return ptst

# def earth2globworld(ptst):
#     track = TrackData()
#     track.header = ptst.header
#     print
#     track.header.frame_id = "global_world"
#     track.position.z = ptst.point.z
#     track.position.x = ptst.point.y
#     track.position.y = ptst.point.x
#     return track

def earth2map(point_stamped):
    pose_map = None
    robot_name = rospy.get_param("~robot_name")
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    while not pose_map and not rospy.is_shutdown():
        # rospy.sleep(0.1)
        try:
            pose_map = tf_buffer.transform(point_stamped, robot_name + rospy.get_param("~map_frame"), rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo("Houston we have a problem with the transforms..." + str(e))

    return pose_map

def start2map(pose):
    if "map" in pose.header.frame_id:
        return pose.pose.pose
    pose_map = None
    robot_name = rospy.get_param("~robot_name")
    # recbot0/odom is very unreliable, assuming recbotX/start approx equals recbotX/odom
    while not pose_map and not rospy.is_shutdown():
        # rospy.sleep(0.1)
        pose_map = transform_pose(pose.pose.pose, robot_name + rospy.get_param("~start_frame"),
                                  robot_name + rospy.get_param("~map_frame"))
    return pose_map

def start2grid(pose, cell_size):
    map_pos = start2map(pose)
    return map2grid(map_pos.position, cell_size)

def grid2map(pos, cell_size):
    return ((pos[1] + 0.5)*cell_size, (pos[0] + 0.5)*cell_size)

def map2grid(pos, cell_size):
    return (int(pos.x / cell_size), int(pos.y / cell_size))

def grid2grid(pos, old_cell_size, new_cell_size):
    return (int(pos[0]*old_cell_size/new_cell_size), int(pos[1]*old_cell_size/new_cell_size))

def costmap2slice(grid_coords, cmap_arr, cell_size):
    # costmap msg because eventually we would want to continually and persistently update within the waypoint planner
    costmap_coords = grid2grid(grid_coords, cell_size, 0.5)
    # print("grid_coords: ", grid_coords)
    # print("costmap_coords: ", costmap_coords)
    bottom_left = (np.array(costmap_coords)).astype(int)
    # print("bottom_left_coords: ", bottom_left)
    top_right = (np.array(costmap_coords) + 2*cell_size).astype(int)
    # print("top_right_coords: ", top_right)
    # print("Costmap dimension inside NATS: ", cmap_arr.shape, " is first dimension bigger?")
    # print("Slicing: ", bottom_left[0]," to ", top_right[0], " and ", bottom_left[1], " to ", top_right[1])
    return cmap_arr[bottom_left[0]:top_right[0], bottom_left[1]:top_right[1]]

# def robogrid2grid()

def init_searchpolygon_marker(publisher):
    '''
    Function will provide the default marker message attributes
    '''
    publisher.msg.header.frame_id = "/{robot_name}/map".format(robot_name=rospy.get_param("~robot_name"))  # Should select a more sensible frame
    publisher.msg.header.stamp = rospy.Time.now()
    publisher.msg.id = 0
    publisher.msg.action = Marker.ADD
    publisher.msg.pose.orientation.w = 1.0
    publisher.msg.type = Marker.LINE_STRIP
    publisher.msg.scale.x = 10.0
    publisher.msg.color.r = 1.0
    publisher.msg.color.a = 1.0
    return publisher

def update_null_observation(pose_map, nats_obj, robot_type, grid_coord, robot_name = None):
    h, l = map2grid(pose_map.position, rospy.get_param("~cell_size"))
    quaternion = pose_map.orientation
    d = quaternion_to_facing_direction(quaternion)

    if grid_coord != (l, h, d):
        nats_obj.robot_path.append((l, h))
        nats_obj.robot_path_timestamps.append(rospy.get_time())
        # print("Updated robot_path: ", len(nats_obj.robot_path))
        # print("Updated robot_path_timestamps: ", len(nats_obj.robot_path_timestamps))
        if robot_name is None:
            robot_name = nats_obj.robot_name
        if nats_obj.robot_path_dict.get(robot_name, None) is None:
            nats_obj.robot_path_dict[robot_name] = [(l, h)]
            nats_obj.robot_path_timestamps_dict[robot_name] = [rospy.get_time()]
        else:
            nats_obj.robot_path_dict[robot_name].append((l, h))
            nats_obj.robot_path_timestamps_dict[robot_name].append(rospy.get_time())
        # Either just starting or grid pos has changed
        # Either way need to update observation list
        grid_coord = (l, h, d)
        x, non_zero_idx, noise_var = nats_obj.create_directional_sensor(l, h, d, robot_type=robot_type)

        if not np.any(nats_obj.points_dict['X']):
            #print("# first time")
            Y = np.zeros((x.shape[0], 2))
            Y[:, 1] = noise_var
            nats_obj.points_dict['X'] = x
            nats_obj.points_dict['Y'] = Y
            nats_obj.points_dict['par'] = [0, np.array([l * h])]
        else:
            # print("# some appending magic")
            Y = np.zeros((x.shape[0], 2))
            Y[:, 1] = noise_var
            nats_obj.points_dict['X'] = np.append(nats_obj.points_dict['X'], x, axis=0)
            nats_obj.points_dict['Y'] = np.append(nats_obj.points_dict['Y'], Y, axis=0)
            nats_obj.points_dict['par'] = [0, np.array([l * h])]
    return grid_coord


def quaternion_to_facing_direction(quaternion):
    euler_angles = euler_from_quaternion(quaternion=[quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    yaw = np.rad2deg(euler_angles[2])
    # Yaw is wrt to the map x axis which is the h axis in grid coordinates
    # Yaw increases counter clockwise
    # d = 0 is E, 1 is N, 2 is W, 3 is S, where map x is N
    d = int((yaw + 360 + 45) / 90) % 4
    return d


# function for line generation
def bresenham_line_gen(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points

def remove_duplicates(seq):
    #preserve order, expects a python list
    seen = set()
    seen_add = seen.add
    return [x for x in seq if not (x in seen or seen_add(x))]

def get_point_on_polygon(position_map, search_polygon_msg):
    search_polygon_msg.polygon.points = remove_duplicates(search_polygon_msg.polygon.points)[:-1]
    nearest_vertex, second_nearest = get_nearest_vertex(position_map, search_polygon_msg, second_nearest=True)
    m2 = -(nearest_vertex[0] - second_nearest[0]) / (nearest_vertex[1] - second_nearest[1])
    c2 = position_map.y - m2 * position_map.x
    c1 = nearest_vertex[1] - (-1 / m2) * nearest_vertex[0]
    x_cand = (c2 - c1) / ((-1 / m2) - m2)
    y_cand = m2 * x_cand + c2
    if ((nearest_vertex[0] <= x_cand <= second_nearest[0]) or (nearest_vertex[0] >= x_cand >= second_nearest[0])) and (
            (nearest_vertex[1] <= y_cand <= second_nearest[1]) or (nearest_vertex[1] >= y_cand >= second_nearest[1])):
        # candidate is actually on the polygon
        return x_cand, y_cand
    else:
        return nearest_vertex

def get_dummy_goal():
    '''
    Puts initial values into the goal object.
    '''
    pathpub = Path()
    pose = Pose()
    pose_with_timestamp = PoseStamped()
    autoactiongoal = AutonomyActionGoal()
    agoal = AutonomyGoal()
    timenow = rospy.Time.now()
    robot_name = rospy.get_param("~robot_name")
    pose.position.x = 29.239440918
    pose.position.y = 3.87919473648
    pose.orientation.z = -0.0758366073252
    pose.orientation.w = 0.997120258038
    pose_with_timestamp.pose = pose
    pose_with_timestamp.header.frame_id = robot_name+rospy.get_param("~goal_output_frame")
    pose_with_timestamp.header.stamp = timenow
    pathpub.poses.append(pose_with_timestamp)
    pathpub.header.stamp = timenow
    pathpub.header.frame_id = robot_name+rospy.get_param("~goal_output_frame")
    agoal.path = pathpub
    agoal.uuid = str(uuid.uuid1())
    agoal.interpolate_between_waypoints = True
    agoal.large_map = rospy.get_param("~large_map_flag")
    autoactiongoal.header.stamp = timenow
    autoactiongoal.goal_id.id = "/{robot_name}/rviz_interface-3-88.388".format(robot_name=robot_name)
    autoactiongoal.goal_id.stamp = timenow
    autoactiongoal.goal = agoal
    # print()
    return agoal

def rgb2gray(rgb):
    if len(rgb.shape) < 3:
        return rgb
    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b

    return gray

def compute_probability_prior(visibility_prior, costmap, n1, n2):
    inv_prob_map = Image.fromarray(255 * visibility_prior).resize((n2, n1)).convert("L")
    median = np.percentile(np.array(inv_prob_map).flatten(), 40)
    mask = inv_prob_map < median
    # prob_map = (mask * (1/inv_prob_map)).astype(np.float32)
    prob_map = (mask * (1.0/np.array(inv_prob_map))).astype(np.float32)
    traversibility_map = Image.fromarray(np.flipud(np.array(costmap.msg.data).astype(np.float32).reshape((costmap.msg.info.height,
                                                                                  costmap.msg.info.width)))).resize((n2, n1)).convert("L")
    reachable_prob_map = (np.array(traversibility_map) < (np.max(traversibility_map)-np.min(traversibility_map))/2) * prob_map
    return reachable_prob_map / np.max(reachable_prob_map)
