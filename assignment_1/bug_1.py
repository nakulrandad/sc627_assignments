#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ============================================================================
"""
Implements Bug1 algorithm

Author: Nakul Randad (https://github.com/nakulrandad)
"""
# ============================================================================

import os
import math
import numpy as np
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import (
    distance,
    computeDistancePointToPolygon,
    computeTangentVectorToPolygon,
    computeDistancePointToSegment,
    computePolygonCentroid,
)


rospy.init_node("bug_1", anonymous=True)

# Initialize client
client = actionlib.SimpleActionClient("move_xy", MoveXYAction)
client.wait_for_server()

# read input file
with open(os.path.join(os.path.dirname(__file__), "input.txt")) as f:
    inputs = f.read().split("\n")
    start = np.array(list(map(float, inputs[0].split(","))))
    goal = np.array(list(map(float, inputs[1].split(","))))
    step_size = float(inputs[2])
    obstacles = []
    for i in inputs[3:]:
        if i == "":
            obstacles.append([])
        else:
            obstacles[-1].append(list(map(float, i.split(","))))

# setting result as initial location
result = MoveXYResult()
result.pose_final.x = start[0]
result.pose_final.y = start[1]
result.pose_final.theta = 0  # in radians (0 to 2pi)
current_pose = np.array([result.pose_final.x, result.pose_final.y])
path = [current_pose.copy()]
# is_near_obs = False


def update_waypoint(dir_vec, wp):
    wp.pose_dest.x = result.pose_final.x + dir_vec[0] * step_size
    wp.pose_dest.y = result.pose_final.y + dir_vec[1] * step_size
    # theta is the orientation of robot in radians (0 to 2pi)
    wp.pose_dest.theta = math.atan2(dir_vec[1], dir_vec[0])
    # send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)
    client.wait_for_result()
    # getting updated robot location
    result_new = client.get_result()
    current_pose[0] = result.pose_final.x = result_new.pose_final.x
    current_pose[1] = result.pose_final.y = result_new.pose_final.y
    path.append(current_pose.copy())


def move_to_goal(goal):
    dir_vec = (goal - current_pose) / np.linalg.norm(goal - current_pose)
    update_waypoint(dir_vec, MoveXYGoal())


def circumnavigate_obs(obstacle, init_pose):
    print(f"\n#####\nCircumnavigating obstacle {obstacle}\n#####\n")
    dir_vec = -np.array(computeTangentVectorToPolygon(obstacle, init_pose))
    init_vec = init_pose - np.array(
        computePolygonCentroid(obstacle)
    )  # vector from centroid to init_pose
    dist = float("inf")
    counter = 0
    while True:
        update_waypoint(dir_vec, MoveXYGoal())
        curr_vec = current_pose - np.array(computePolygonCentroid(obstacle))
        dir_vec = -np.array(
            computeTangentVectorToPolygon(obstacle, current_pose)
        )
        # print(f"Direction vector: {dir_vec}")
        if distance(current_pose, goal) < dist:
            obs_leave_pose = current_pose.copy()
            dist = distance(current_pose, goal)
            # print(f"Minimum dist to goal: {dist} from {obs_leave_pose}")
        # print(f"obs_leave_pose: {obs_leave_pose}")
        rot_lim = np.dot(curr_vec, init_vec) / (
            np.linalg.norm(curr_vec) * np.linalg.norm(init_vec)
        )
        counter += 1
        if counter > 10 and rot_lim > 0.95:
            print(f"Circumnavigation ends!")
            break
    return obs_leave_pose


def depart_obs(obstacle, obs_leave_pose):
    print(f"\n#####\nMoving towards {obs_leave_pose}\n#####\n")
    obs_leave_vec = obs_leave_pose - np.array(computePolygonCentroid(obstacle))
    obs_leave_vec /= np.linalg.norm(obs_leave_vec)
    dir_vec = -np.array(computeTangentVectorToPolygon(obstacle, current_pose))
    counter = 0
    while True:
        update_waypoint(dir_vec, MoveXYGoal())
        curr_vec = current_pose - np.array(computePolygonCentroid(obstacle))
        dir_vec = -np.array(
            computeTangentVectorToPolygon(obstacle, current_pose)
        )
        rot_lim = np.dot(curr_vec, obs_leave_vec) / np.linalg.norm(curr_vec)
        counter += 1
        if counter > 10 and rot_lim > 0.95:
            print(f"Departed from obstacle {obstacle}")
            break


print("\n#####\nMoving towards goal\n#####\n")
while distance(current_pose, goal) > step_size:
    dist = float("inf")
    for i, obstacle in enumerate(obstacles):
        d = computeDistancePointToPolygon(obstacle, current_pose)
        if d < dist:
            dist = d
            closest_obs_id = i
    # print(f"Closest obstacle is: {closest_obs_id}")

    # print(f"Distance to obs {i}: {dist}")
    dir_vec = (goal - current_pose) / np.linalg.norm(goal - current_pose)
    dist_to_polygon_frd = computeDistancePointToPolygon(
        obstacles[closest_obs_id], current_pose + dir_vec * step_size * 0.8
    )
    if dist_to_polygon_frd < step_size:
        obs_leave_node = circumnavigate_obs(
            obstacles[closest_obs_id], current_pose
        )
        # print(f"obs_leave_node: {obs_leave_node}")
        depart_obs(obstacles[closest_obs_id], obs_leave_node)
        move_to_goal(goal)
    else:
        move_to_goal(goal)


print("\n#####\nGoal has been achieved!\n#####\n")

with open(os.path.join(os.path.dirname(__file__), "output_1.txt"), "w") as f:
    for waypoint in path:
        f.write(f"{waypoint[0]},{waypoint[1]}\n")
