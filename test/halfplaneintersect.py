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
        # If this half-plane already contains the current point, all is well.
        if dot(point - line.point, line.direction) >= 0:
            # assert False, point
            continue
        prev_lines = itertools.islice(lines, i)
        left_dist, right_dist, flag = line_halfplane_intersect(line, prev_lines)

        if flag == 1:
            point = point_line_project(line, optimal_point, left_dist, right_dist)
            # log.write(f"可行解为{point}\n")
        elif cnt == 1: 
            return None
    return point

def point_line_project(line, point, left_bound, right_bound):
    # print("left_bound=%s, right_bound=%s" % (left_bound, right_bound))
    new_dir = perp(line.direction)
    # print("new_dir=%s" % new_dir)
    proj_len = dot(point - line.point, new_dir)
    # print("proj_len=%s" % proj_len)
    clamped_len = clip(proj_len, left_bound, right_bound)
    # print("clamped_len=%s" % clamped_len)
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
            # The half-planes are parallel.
            if num < 0:
                # The intersection of the half-planes is empty; there is no
                # solution.
                # raise InfeasibleError
                flag = 0
                return flag, flag, flag
            else:
                # The *half-planes* intersect, but their lines don't cross, so
                # ignore.
                continue

        offset = num / den
        if den > 0:
            # Point of intersection is to the right.
            right_dist = min((right_dist, offset))
        else:
            # Point of intersection is to the left.
            left_dist = max((left_dist, offset))

        if left_dist > right_dist:
            # The interval is inconsistent, so the feasible region is empty.
            flag = 0
            return left_dist, right_dist, flag
            # raise InfeasibleError
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

    # a = point_line_project(lines[0], point, -20, 20)
    # print((a - lines[0].point)/(-perp(lines[0].direction)))
    # print(a)

    a = point_line_project(lines[1], point, -10000, -3)
    # print((a - lines[1].point)/(-perp(lines[1].direction)))
    print(a)
