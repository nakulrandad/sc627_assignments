#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ============================================================================
"""
Implements Bug1 algorithm described in E1.8

Author: Nakul Randad (https://github.com/nakulrandad)
"""
# ============================================================================

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib

# import other helper files if any


rospy.init_node("test", anonymous=True)

# Initialize client
client = actionlib.SimpleActionClient("move_xy", MoveXYAction)
client.wait_for_server()

# read input file


# setting result as initial location
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0  # in radians (0 to 2pi)

while True:  # replace true with termination condition

    # determine waypoint based on your algo
    # this is a dummy waypoint (replace the part below)
    wp = MoveXYGoal()
    wp.pose_dest.x = 1 + result.pose_final.x
    wp.pose_dest.y = 0
    # theta is the orientation of robot in radians (0 to 2pi)
    wp.pose_dest.theta = 0

    # send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)

    client.wait_for_result()

    # getting updated robot location
    result = client.get_result()

    # write to output file (replacing the part below)
    print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)
