#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ============================================================================
"""
Show the path followed by robot

Author: Nakul Randad (https://github.com/nakulrandad)
"""
# ============================================================================
import os
import numpy as np
import matplotlib.pyplot as plt


def readInputs(filename="input.txt"):
    # read input file
    with open(os.path.join(os.path.dirname(__file__), filename)) as f:
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
    return start, goal, step_size, obstacles


def readPath(filename="output.txt"):
    # read output file
    with open(os.path.join(os.path.dirname(__file__), filename)) as f:
        outputs = f.read().split("\n")
        path = []
        path.append([])
        for i in outputs:
            if i == "":
                continue
            path[-1].append(list(map(float, i.split(","))))
    return path


def showPath(
    isFigSaved=False,
    inputFile="input.txt",
    outputBaseFile="output_base.txt",
    output1File="output_1.txt",
    dist_to_goal_arr=None,
):
    start, goal, _, obstacles = readInputs(inputFile)
    path_base = readPath(outputBaseFile)
    path_1 = readPath(output1File)

    plt.figure()
    # plt.grid()
    plt.axis("equal")
    plt.plot(
        *np.array(path_base).T, "--", color="magenta", label="Path by bug base"
    )
    plt.plot(*np.array(path_1).T, color="black", label="Path by bug 1")
    plt.plot(start[0], start[1], "bo", markersize=8, label="Start")
    plt.plot(goal[0], goal[1], "go", markersize=8, label="Goal")
    plt.legend(loc="lower right")

    for i in range(len(obstacles)):
        t1 = plt.Polygon(obstacles[i], color="red", fill=True)
        plt.gca().add_patch(t1)

    if isFigSaved:
        plt.savefig(
            os.path.join(os.path.dirname(__file__), "results", "path.png")
        )

    plt.show()

    if dist_to_goal_arr != None:
        plt.figure()
        plt.grid()
        plt.xlabel("Time (s)")
        plt.ylabel("Distance to goal (m)")
        plt.plot(*np.array(dist_to_goal_arr).T)
        if isFigSaved:
            plt.savefig(
                os.path.join(
                    os.path.dirname(__file__), "results", "dist_to_goal.png"
                )
            )
        plt.show()


if __name__ == "__main__":
    showPath(True)
