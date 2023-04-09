'''
Author: BigCiLeng && bigcileng@outlook.com
Date: 2023-03-30 13:54:18
LastEditors: BigCiLeng && bigcileng@outlook.com
LastEditTime: 2023-04-09 13:05:07
FilePath: \HUAWEIRobot\test\halfplaneintersect.py
Description: 

Copyright (c) 2023 by bigcileng@outlook.com, All Rights Reserved. 
'''
from __future__ import division

import itertools

from numpy import dot, clip, array, sqrt
from numpy.linalg import det

class InfeasibleError(RuntimeError):
    pass


class Line(object):
    def __init__(self, point, direction):
        super(Line, self).__init__()
        self.point = array(point)
        self.direction = normalized(array(direction))

    def __repr__(self):
        return "Line(%s, %s)" % (self.point, self.direction)


def halfplane_optimize(lines, optimal_point, cnt):
    point = optimal_point
    for i, line in enumerate(lines):
        if dot(point - line.point, line.direction) >= 0:
            continue
        prev_lines = itertools.islice(lines, i)
        left_dist, right_dist, flag = line_halfplane_intersect(line, prev_lines)

        if flag == 1:
            point = point_line_project(line, optimal_point, left_dist, right_dist)
        elif cnt == 1: 
            return None
    return point

def point_line_project(line, point, left_bound, right_bound):
    new_dir = perp(line.direction)
    proj_len = dot(point - line.point, new_dir)
    clamped_len = clip(proj_len, left_bound, right_bound)
    return line.point + new_dir * clamped_len

def line_halfplane_intersect(line, other_lines):
    left_dist = float("-inf")
    right_dist = float("inf")
    flag = 1
    for prev_line in other_lines:
        num1 = dot(prev_line.direction, line.point - prev_line.point)
        den1 = det((line.direction, prev_line.direction))

        num = num1
        den = den1
        if den == 0:
            if num < 0:
                flag = 0
                return flag, flag, flag
            else:
                continue

        offset = num / den
        if den > 0:
            right_dist = min((right_dist, offset))
        else:
            left_dist = max((left_dist, offset))

        if left_dist > right_dist:
            flag = 0
            return left_dist, right_dist, flag
    return left_dist, right_dist, flag

def perp(a):
    return array((a[1], -a[0]))

def norm_sq(x):
    return dot(x, x)

def norm(x):
    return sqrt(norm_sq(x))

def normalized(x):
    l = norm_sq(x)
    assert l > 0, (x, l)
    return x / sqrt(l)

if __name__ == '__main__':
    lines = [
        Line((-2, 0), (-1, 1)),
        Line((0, -1), (1, 0))
    ]
    point = array((1, 0))
    result = halfplane_optimize(lines, point)
    print(result, norm(result))


    a = point_line_project(lines[1], point, -10000, -3)
    print(a)
