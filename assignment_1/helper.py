#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ============================================================================
"""
This file contains all the functions (and more if necessary) mentioned
in E1.6, E1.7

Author: Nakul Randad (https://github.com/nakulrandad)
"""
# ============================================================================

import math


def distance(p1, p2):
    """
    Calculates the distance between two points
    :param p1: Point 1
    :param p2: Point 2
    :return: Distance between the two points
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def computeLineThroughTwoPoints(p1, p2):
    """
    Computes the line through two points
    :param p1: Point 1
    :param p2: Point 2
    :return: Line through the two points
    """
    norm_fact = distance(p1, p2)  # ensures a^2 + b^2 = 1
    if norm_fact < 10 ** -8:
        return Exception("Coincident points received")
    a, b, c = (
        (p1[1] - p2[1]) / norm_fact,
        (p2[0] - p1[0]) / norm_fact,
        (p1[0] * p2[1] - p2[0] * p1[1]) / norm_fact,
    )
    return a, b, c


def computeDistancePointToLine(q, p1, p2):
    """
    Computes the distance of a point to a line
    :param q: Point
    :param p1: Point 1
    :param p2: Point 2
    :return: Distance of the point to the line
    """
    a, b, c = computeLineThroughTwoPoints(p1, p2)
    return abs(a * q[0] + b * q[1] + c)


def computeDistancePointToSegment(q, p1, p2):
    """
    Computes the distance of a point to a segment
    :param q: Point
    :param p1: Point 1
    :param p2: Point 2
    :return: Distance of the point to the segment and state of the point
    (0: on segment, 1: left of segment, 2: right of segment)
    """
    vec1 = (p2[0] - p1[0], p2[1] - p1[1])  # vector from p1 to p2
    vec2 = (p1[0] - q[0], p1[1] - q[1])  # vector from q to p1
    rel_length = (vec1[0] * vec2[0] + vec1[1] * vec2[1]) / (
        vec1[0] ** 2 + vec1[1] ** 2
    )  # dot product divided by norm squared
    if rel_length < 0:
        dist = distance(q, p1)
        state = 1
    elif rel_length > 1:
        dist = distance(q, p2)
        state = 2
    else:
        dist = computeDistancePointToLine(q, p1, p2)
        state = 0
    return dist, state


def computeDistancePointToPolygon(P, q):
    """
    Computes the distance of a point to a polygon
    :param P: Polygon {an array with n rows and 2 columns}
    :param q: Point
    :return: Distance of the point to the polygon
    """
    dist = float("inf")
    for i in range(len(P) - 1):
        d, _ = computeDistancePointToSegment(q, P[i], P[i + 1])
        if d < dist:
            dist = d
    return dist


def computeTangentVectorToPolygon(P, q):
    """
    Computes the tangent vector to a polygon
    :param P: Polygon {an array with n rows and 2 columns}
    :param q: Point
    :return: Tangent vector to the polygon
    """
    dist = float("inf")
    for i in range(len(P) - 1):
        d, _ = computeDistancePointToSegment(q, P[i], P[i + 1])
        if d < dist:
            dist = d
            closest_segment = [P[i], P[i + 1]]
    _, state = computeDistancePointToSegment(q, *closest_segment)
    if state == 0:
        #! check for sense of rotation
        tangent = [
            (closest_segment[1][0] - closest_segment[0][0])
            / distance(*closest_segment),
            (closest_segment[1][1] - closest_segment[0][1])
            / distance(*closest_segment),
        ]
    elif state == 1:
        tangent = [
            -q[1] + closest_segment[0][1],
            q[0] - closest_segment[0][0],
        ] / distance(closest_segment[0], q)
    else:
        tangent = [
            (-q[1] + closest_segment[1][1]) / distance(closest_segment[1], q),
            (q[0] - closest_segment[1][0]) / distance(closest_segment[1], q),
        ]
