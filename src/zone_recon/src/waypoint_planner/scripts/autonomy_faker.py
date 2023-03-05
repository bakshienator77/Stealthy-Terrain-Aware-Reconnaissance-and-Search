#!/usr/bin/env python
from __future__ import print_function


"""
Purpose of this code is to pretend to be autonomy and hence should provide costmap, local pose
"""

import rospy
from waypoint_planner.a_star_searching_from_two_side import load_boundary_and_obstacles_from_image, searching_control, \
    random_coordinate
from waypoint_planner.waypoint_planner_utils import SendMessage, grid2map, map2grid, GetMessage, rgb2gray, costmap2slice
from nav_msgs.msg import OccupancyGrid, Odometry
import actionlib
import os
from matplotlib import image
from matplotlib import pyplot as plt
from PIL import Image
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_from_matrix
import math
from anon_msgs.msg import AutonomyActionGoal, AutonomyGoal, AutonomyAction
import threading
from std_msgs.msg import Float32MultiArray

# threading_lock = threading.Lock()


class AutonomyServer:
    def __init__(self, topic, action):
        robot_name = rospy.get_param("~robot_name")
        self.server = actionlib.SimpleActionServer(topic, action, self.execute, False)
        self.server.start()
        self.cell_size = rospy.get_param("~cell_size")
        self.costmap_publisher = SendMessage(topic=rospy.get_param("~costmap_update_topic"), msg_type=OccupancyGrid,
                                             latched=True)
        self.traversability_costmap_load_and_publish()
        self.visibility_costmap = None
        self.search_mode = rospy.get_param("~search_mode")
        if rospy.get_param("~visibility_cost_weightage") != 0 and rospy.get_param("~search_mode") == "nats":
            rospy.loginfo("SOME VERSION OF STAR RUNNING ON: " + robot_name + " STAR: " + str(
                rospy.get_param("~visibility_cost_weightage")))
            # Only if visibility cost weightage is non zero
            self.visibility_map_subscriber = GetMessage(rospy.get_param("~viz_costmap_topic_lowres"), Float32MultiArray,
                                                        callback_modifier=self.msg_to_viz_costmap,
                                                        # threading_lock=threading_lock
                                                        )
            self.set_visibility_prior()

        self.local_pose_publisher = SendMessage(topic=rospy.get_param("~local_pose_topic"), msg_type=Odometry,
                                                latched=True)
        direction, start = self.init_start_pose(robot_name)
        self.publish_local_pose_from_grid_coords(direction, start)
        self.waypoint_number = 0
        self.logging_path = None

    def set_visibility_prior(self):
        working_dir = "/home/" + str(os.environ['USER']) + "/src/zone_recon/"
        visibility_prior = image.imread(working_dir + "assets/" + \
                                        rospy.get_param("~location") + "/visibility_cost_" + \
                                        rospy.get_param("~map_type") + ".png")
        self.visibility_prior = np.flipud(
            np.array(Image.fromarray(255 * visibility_prior).resize((self.n2, self.n1)).convert("L"))).T
        rospy.loginfo("Visibility prior dimensions " + str(self.visibility_prior.shape))
        # self.visibility_prior /= np.max(self.visibility_prior)
        # print("VISIBILITY PRIOR UNNORMALIZED ", np.max(self.visibility_prior), np.min(self.visibility_prior))

    def msg_to_viz_costmap(self, float32multiarray_msg):
        # rospy.loginfo("\n\n\nYOU HAVE ENTERED THE VISIBILITY COSTMAP CALLBACK\n\n\n")
        data_vector = np.array(float32multiarray_msg.data).flatten()

        if sum(data_vector) == 0:
            with_prior = self.visibility_prior
        else:
            reorient_grid = data_vector.reshape((self.n2, self.n1))
            # print("Shape of repeated grid: ", reorient_grid.shape)
            # print("Shape needed: ", self.n1, self.n2)
            with_prior = reorient_grid / np.max(reorient_grid)
            with_prior[with_prior == 0] = 0.05
            # print("Shape of visibility prior: ", self.visibility_prior.shape)
            with_prior = with_prior * self.visibility_prior

        clipped_grid = with_prior # / np.max(with_prior)
        # clipped_grid = 100 * np.flipud(visibility_prior)
        print("Shape of clipped grid: ", clipped_grid.shape)
        print("Max of clipped grid: ", np.max(clipped_grid), "min of clipped grid: ", np.min(clipped_grid))
        self.visibility_costmap = clipped_grid
        return

    def normalize_ptp(self, loss):
        if np.ptp(loss) != 0:
            return (loss - np.min(loss)) / np.ptp(loss)
        else:
            return loss

    def init_start_pose(self, robot_name):
        # start = random_coordinate(self.bottom_vertex, self.top_vertex, rospy.get_param("~random_seed") + sum([ord(c) for c in robot_name]))
        direction = 0
        start = [14, 5]
        # while start in self.obstacle:
        #     print("whoops")
        #     start = random_coordinate(self.bottom_vertex, self.top_vertex, None)
        return direction, start

    def publish_local_pose_from_grid_coords(self, direction, start):
        local_pose_msg = Odometry()
        local_pose_msg.header.stamp = rospy.Time.now()
        local_pose_msg.header.frame_id = rospy.get_param("~robot_name") + rospy.get_param("~map_frame")
        coords = grid2map(start, cell_size=self.cell_size)
        # print(coords)
        # Transform to map coords
        local_pose_msg.pose.pose.position.x = coords[1]
        local_pose_msg.pose.pose.position.y = coords[0]
        quaternion = quaternion_from_euler(0, 0, direction * 1.57)
        # print("grid direction: ", result['grid_pos'][2], " corresponds to quaternion in map frame: ", quaternion)
        local_pose_msg.pose.pose.orientation.x = quaternion[0]
        local_pose_msg.pose.pose.orientation.y = quaternion[1]
        local_pose_msg.pose.pose.orientation.z = quaternion[2]
        local_pose_msg.pose.pose.orientation.w = quaternion[3]
        self.local_pose_publisher.msg = local_pose_msg
        self.local_pose_publisher.publish()

    def set_allowable_grid_locs(self, cmap_arr, costmap_vacancy_threshold, grid_cell_size, robot_type):
        costmap_cell_size = rospy.get_param("~costmap_cell_size")
        # cmap_arr = cmap_arr.astype(np.float)
        # plt.imshow(cmap_arr)
        # print(cmap_arr)
        # plt.savefig("/home/nabakshi/src/zone_recon/temporary_debug_costmap.png")
        # plt.close()
        costmap_array = 255 * np.ones((self.n2, self.n1))
        for h in range(0, self.n2):
            for l in range(0, self.n1):
                for d in [0]:  # direction of travel, NSEW, I should use NE,SW,etc.
                    if robot_type == "ugv":
                        cmap_slice = costmap2slice((l, h), cmap_arr, grid_cell_size)
                        if float(np.count_nonzero(cmap_slice))/(cmap_slice.shape[0] * cmap_slice.shape[1]) > (1 - costmap_vacancy_threshold):
                            # If the non-zero cost cells exceed a certain threshold a.k.a not freespace cells
                            continue
                    costmap_array[h,l] = 0
        return costmap_array

    def traversability_costmap_load_and_publish(self):
        cell_size = self.cell_size
        costmap_vacancy_threshold = rospy.get_param("~costmap_vacancy_threshold")

        working_dir = "/home/" + str(os.environ['USER']) + "/src/autonomy/"
        img = np.flipud(rgb2gray(image.imread(
            working_dir + "autonomy_assets/locations/" + rospy.get_param("~location") + "/map/" + rospy.get_param(
                "~map_type") + "_map.png")))
        # print("LOOOOOOOOOK HERE: ", len((100 * (img.flatten() < 0.5)).tolist()))
        costmap_msg = OccupancyGrid()
        costmap_msg.data = (100 * (img.flatten() < 0.5)).tolist()
        costmap_msg.header.stamp = rospy.Time.now()
        costmap_msg.header.frame_id = rospy.get_param("~robot_name") + rospy.get_param("~map_frame")
        costmap_msg.info.resolution = rospy.get_param(
            "~costmap_cell_size")
        costmap_msg.info.height = img.shape[0]
        costmap_msg.info.width = img.shape[1]
        self.costmap_publisher.msg = costmap_msg
        self.costmap_publisher.publish()

        if rospy.get_param("~robot_type") == "ugv":
            self.n1 = int(np.ceil(
                img.shape[0] * rospy.get_param(
                    "~costmap_cell_size") / cell_size))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
            self.n2 = int(np.ceil(
                img.shape[1] * rospy.get_param(
                    "~costmap_cell_size") / cell_size))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
        if rospy.get_param("~robot_type") == "uas":
            self.n1 = int(np.ceil(rospy.get_param("~costmap_cell_size") * rospy.get_param("~map_height") / (
                cell_size)))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
            self.n2 = int(np.ceil(rospy.get_param("~costmap_cell_size") * rospy.get_param("~map_width") / (
                cell_size)))  # by default each cell in grid is 0.5m x 0.5m, we want it to be cell_size x cell_size
        rospy.loginfo(
            "AUTONOMY FAKER: Grid-size:%dx%d with each cell %dmx%dm" % (self.n1, self.n2, cell_size, cell_size))
        # self.costmap_array = np.array(
        #     Image.fromarray(255 * img.astype(np.float)).resize((self.n2, self.n1)).convert("L")).T
        self.costmap_array = self.set_allowable_grid_locs(img, costmap_vacancy_threshold, self.cell_size, rospy.get_param("~robot_type"))
        self.top_vertex = self.costmap_array.shape
        self.bottom_vertex = (0, 0)
        self.bound, self.obstacle = load_boundary_and_obstacles_from_image(self.costmap_array)

    def execute(self, goal):
        # plan path and return here
        #
        self.waypoint_number += 1

        if self.logging_path is None:
            self.logging_path = rospy.get_param("~logging_path")
            # plt.imshow(self.costmap_array)
            # plt.savefig(self.logging_path + "temporary_debug_costmap.png")
            # plt.close()

        # plt.imshow(self.visibility_prior)
        # plt.savefig(self.logging_path + "temporary_debug_viz_costmap_prior.png")
        # plt.close()

        print(goal)
        end = list(map2grid(goal.path.poses[0].pose.position, self.cell_size))
        start = list(map2grid(self.local_pose_publisher.msg.pose.pose.position, self.cell_size))
        rospy.loginfo("Goal point received, desired path: " + str(start) + " -> " + str(end))
        # print("START: ", start, "END: ", end)
        if start in self.obstacle or end in self.obstacle:
            # goal point not reachable
            rospy.loginfo("Goal point not reachable because goal is on obstacle.")
            self.server.set_aborted()
            return
        # print()
        path = searching_control(start, end, self.bound, self.obstacle, costmap=self.visibility_costmap,
                                 viz_marker=self.logging_path + str(self.waypoint_number))
        # print(path)
        if path is None:
            # goal point not reachable
            rospy.loginfo("Goal point not reachable because path planning failed.")
            self.server.set_aborted()
            return
        print("Waypoint number ", self.waypoint_number, "path following!")
        self.follow_path(path)

        self.server.set_succeeded()
        return

    def follow_path(self, path):
        d = 0
        print(self.search_mode)
        if "random" in self.search_mode or "coverage" in self.search_mode:
            rate = rospy.Rate(1)
        else:
            rate = rospy.Rate(1)
        for i in range(len(path) - 1):
            loc = path[i]
            # d = 0 is E, 1 is N, 2 is W, 3 is S, where map x is N
            dy = path[i + 1][1] - path[i][1]
            dx = path[i + 1][0] - path[i][0]
            yaw = math.atan2(dy, dx)
            yaw = np.rad2deg(yaw)
            d = int((yaw + 360 + 45) / 90) % 4
            self.publish_local_pose_from_grid_coords(d, loc)
            rate.sleep()
            print(loc, d)
        self.publish_local_pose_from_grid_coords(d, path[-1])


if __name__ == '__main__':
    rospy.loginfo("AUTONOMY FAKER HAS STARTED")
    rospy.init_node('autonomy_faker')
    # need to have the other end of the navigation manager 
    # load cost map

    autonomy_server_topic = rospy.get_param("~autonomy_server_topic")
    autonomy_server = AutonomyServer(autonomy_server_topic, AutonomyAction)
    # rospy.spin()
    # push costmap to necessary topic
    # setup the action client at the other end
    # then plan
    # then iterate through the given path and broadcast localpose
