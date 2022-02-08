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
)


rospy.init_node("bug_base", anonymous=True)

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
path = [current_pose]
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
    result = client.get_result()
    current_pose = np.array([result.pose_final.x, result.pose_final.y])


def move_to_goal(current_pose):
    dir_vec = (goal - current_pose) / np.linalg.norm(goal - current_pose)
    update_waypoint(dir_vec, MoveXYGoal())
    
    
def circumnavigate_obs(obstacle, init_pose):
    dir_vec = -np.array(computeTangentVectorToPolygon(obstacle, init_pose))
    dist = float("inf")
    while True:
        update_waypoint(dir_vec, MoveXYGoal())
        dir_vec = -np.array(computeTangentVectorToPolygon(obstacle, current_pose))
        if distance(current_pose, goal) < dist:
            obs_leave_node = current_pose
            dist = distance(current_pose, goal)
        if distance(current_pose, init_pose) < step_size:
            break
    return obs_leave_node


def depart_obs(obstacle, obs_leave_node)
    dir_vec = -np.array(computeTangentVectorToPolygon(obstacle, current_pose))
    while True:
        update_waypoint(dir_vec, MoveXYGoal())
        dir_vec = -np.array(computeTangentVectorToPolygon(obstacle, current_pose))
        if distance(current_pose, obs_leave_node) < step_size:
            break


while distance(current_pose, goal) > step_size:
    dist = float("inf")
    for i, obstacle in enumerate(obstacles):
        d = computeDistancePointToPolygon(obstacle, current_pose)
        if d < dist:
            dist = d
            closest_obs_id = i

    # print(f"Distance to obs {i}: {dist}")

    dist_to_polygon_frd = computeDistancePointToPolygon(
        obstacles[closest_obs_id], current_pose + dir_vec * step_size * 0.8
    )
    if dist_to_polygon_frd < step_size:
        init_pose = current_pose
        dir_vec = -np.array(
            computeTangentVectorToPolygon(
                obstacles[closest_obs_id], current_pose
            )
        )

    # write to output file (replacing the part below)
    path.append(current_pose)
    # print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)
    # print(f"Distance to goal: {distance(current_pose, goal)}")

print("Goal has been achieved!")

# with open(
#     os.path.join(os.path.dirname(__file__), "output_base.txt"), "w"
# ) as f:
#     for waypoint in path:
#         f.write(f"{waypoint[0]},{waypoint[1]}\n")
