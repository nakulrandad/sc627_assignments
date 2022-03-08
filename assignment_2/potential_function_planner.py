#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ============================================================================
"""
SC 627 Assignment 2: Path planning by potential functions

Author: Nakul Randad (https://github.com/nakulrandad)
"""
# ============================================================================

import os
import sys
import math
import numpy as np
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from path_viewer import showPath

sys.path.append(os.path.join(os.path.dirname(__file__), "../assignment_1"))
from helper import (
    distance,
    computeDistancePointToPolygon,
    computeNormalVectorToPolygon,
)


def ROS_init():
    rospy.init_node("potential_func_planner", anonymous=True)

    # Initialize client
    client = actionlib.SimpleActionClient("move_xy", MoveXYAction)
    client.wait_for_server()
    return client


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

client = ROS_init()

# setting result as initial location
result = MoveXYResult()
result.pose_final.x = start[0]
result.pose_final.y = start[1]
result.pose_final.theta = 0  # in radians (0 to 2pi)
current_pose = np.array([result.pose_final.x, result.pose_final.y])
path = [current_pose.copy()]


def update_waypoint(dir_vec):
    wp = MoveXYGoal()
    wp.pose_dest.x = result.pose_final.x + dir_vec[0] * step_size
    wp.pose_dest.y = result.pose_final.y + dir_vec[1] * step_size
    # theta is the orientation of robot in radians (0 to 2pi)
    wp.pose_dest.theta = math.atan2(dir_vec[1], dir_vec[0])
    # print("Sending waypoint:\n", wp.pose_dest, "\n")
    # send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)
    client.wait_for_result()
    # getting updated robot location
    result_new = client.get_result()
    current_pose[0] = result.pose_final.x = result_new.pose_final.x
    current_pose[1] = result.pose_final.y = result_new.pose_final.y
    path.append(current_pose.copy())


def computeGrad():
    dstar = 1.5
    chi = 1.4
    Qstar = 1.5
    eta = 1.0

    if distance(current_pose, goal) <= dstar:
        attractive_grad = (goal - current_pose) * chi
    else:
        attractive_grad = (
            (goal - current_pose) * dstar * chi / distance(current_pose, goal)
        )

    repulsive_grad = np.array([0, 0])
    for obstacle in obstacles:
        dist = computeDistancePointToPolygon(obstacle, current_pose)
        if dist < Qstar:
            normal = computeNormalVectorToPolygon(obstacle, current_pose)
            repulsive_grad = repulsive_grad + np.array(normal) * eta * (
                -1 / Qstar + 1 / dist
            ) * (1 / dist ** 2)

    grad = attractive_grad - repulsive_grad
    grad /= np.linalg.norm(grad)
    return grad


print("\n#####\nStarting to move towards goal\n#####\n")

while distance(current_pose, goal) >= step_size:
    closeness_threshold = 0.3
    if distance(current_pose, goal) <= closeness_threshold:
        dir_vec = goal - current_pose
        dir_vec /= np.linalg.norm(dir_vec)
        update_waypoint(dir_vec)
    else:
        dir_vec = computeGrad()
        update_waypoint(dir_vec)

print("\n#####\nGoal has been achieved!\n#####\n")

with open(os.path.join(os.path.dirname(__file__), "output.txt"), "w") as f:
    for waypoint in path:
        f.write(f"{waypoint[0]},{waypoint[1]}\n")

# showPath()