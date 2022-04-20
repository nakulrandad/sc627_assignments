#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ============================================================================
"""
Assignment 4: Robotic Networks in Balancing

Author: Nakul Randad (https://github.com/nakulrandad)
"""
# ============================================================================

import math
import time
import numpy as np
import matplotlib.pyplot as plt

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


ANG_MAX = math.pi / 18
VEL_MAX = 0.15

path_x = []
path_y = []
time_arr = []

start_time = time.time()

coord = [0.0, 0.0, 0.0]
coord_left = [0.0, 0.0, 0.0]
coord_right = [0.0, 0.0, 0.0]

vel = 0.0
v_left = 0.0
v_right = 0.0

omega = 0.0

eps = 0.001
max_itera = 1000


def velocity_convert(x, y, theta, vel_x, vel_y):
    """
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    """
    v_lin = min(max(vel_x, -VEL_MAX), VEL_MAX)
    v_ang = 0.0
    return v_lin, v_ang


def callback_odom(data):
    """
    Get robot data
    """
    print(data)

    global vel, omega

    coord[0] = data.pose.pose.position.x
    coord[1] = data.pose.pose.position.y
    vel = data.twist.twist.linear.x
    omega = data.twist.twist.angular.z
    coord[2] = euler_from_quaternion(
        [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        ]
    )[2]
    path_x.append(coord[0])
    path_y.append(coord[1])
    time_arr.append(time.time() - start_time)


def callback_left_odom(data):
    """
    Get left robot data
    """
    print("left robot")
    print(data)

    global v_left

    coord_left[0] = data.pose.pose.position.x
    coord_left[1] = data.pose.pose.position.y
    v_left = data.twist.twist.linear.x
    coord_left[2] = euler_from_quaternion(
        [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        ]
    )[2]


def callback_right_odom(data):
    """
    Get right robot data
    """
    print("right robot")
    print(data)

    global v_right

    coord_right[0] = data.pose.pose.position.x
    coord_right[1] = data.pose.pose.position.y
    v_right = data.twist.twist.linear.x
    coord_right[2] = euler_from_quaternion(
        [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        ]
    )[2]


if __name__ == "__main__":
    rospy.init_node("assign4_balancing", anonymous=True)
    rospy.Subscriber("/odom", Odometry, callback_odom)  # topic name fixed
    rospy.Subscriber(
        "/left_odom", Odometry, callback_left_odom
    )  # topic name fixed
    rospy.Subscriber(
        "/right_odom", Odometry, callback_right_odom
    )  # topic name fixed

    pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    r = rospy.Rate(30)

    itera = 0
    while not rospy.is_shutdown():
        k = 1
        u = k * (coord_left[0] - coord[0]) + k * (coord_right[0] - coord[0])
        theta = math.pi if np.sign(u) < 0 else 0

        cmd_v_x = u
        cmd_v_y = 0

        # convert velocity vector to linear and angular velocties using velocity_convert function given above
        v_lin, v_ang = velocity_convert(None, None, theta, cmd_v_x, cmd_v_y)

        # publish the velocities below
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        pub_vel.publish(vel_msg)
        r.sleep()

        # Conditions to exit the loop
        is_vel_low = (
            abs(vel) < eps and abs(v_left) < eps and abs(v_right) < eps
        )
        if is_vel_low and itera > max_itera:
            break

        itera += 1

    # Plots
    fig, axs = plt.subplots(2, 1, figsize=(8, 8))

    axs[0].plot(path_x, path_y)
    axs[0].set_ylabel("y-axis")
    axs[0].set_xlabel("x-axis")
    axs[0].set_title("Robot Trajectory")
    axs[0].grid()

    axs[1].plot(time_arr, path_x)
    axs[1].set_ylabel("Position along X")
    axs[1].set_xlabel("Time")
    axs[1].set_title("Position(X) vs. time")
    axs[1].grid()

    fig.tight_layout()
    plt.show()
