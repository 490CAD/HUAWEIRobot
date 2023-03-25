import numpy as np
from numpy import array, sqrt, copysign, dot
from numpy.linalg import det
from halfplaneintersect import halfplane_optimize, Line, perp
from config import CFG


cfg = CFG()

class Agent(object):
    """A disk-shaped agent."""
    def __init__(self, position, velocity, radius, max_speed, pref_velocity):
        super(Agent, self).__init__()
        self.position = array(position)
        self.velocity = array(velocity)
        self.radius = radius
        self.max_speed = max_speed
        self.pref_velocity = array(pref_velocity)


def orca(robot_id, robots, t, dt, pid_list):
    robot_next_state = []

    for k, i in enumerate(pid_list):
        new_speed = i[1]
        new_toward = robots[k].toward
        new_toward = i[0] * 0.02 + new_toward
        if new_toward >= cfg.PI:
            new_toward -= 2 * cfg.PI
        elif new_toward < -cfg.PI:
            new_toward += 2 * cfg.PI
        robot_next_state.append([new_toward, new_speed])
    lines = []
    flag = 1
    if robot_next_state[robot_id][1] < 0:
        flag = -1
    v_x = robot_next_state[robot_id][1] * np.cos(robot_next_state[robot_id][0]) * flag
    v_y = -robot_next_state[robot_id][1] * np.sin(robot_next_state[robot_id][0]) * flag


    # robots[robot_id].value = cfg.THING_VALUE[robots[robot_id].take_thing] * robots[robot_id].time_f * robots[robot_id].crush_f
    for collider in robots[0: robot_id] + robots[robot_id + 1:]:

        # collider.value = cfg.THING_VALUE[collider.take_thing] * collider.time_f * collider.crush_f
        dv, n = get_avoidance_velocity(robots[robot_id], collider, t, dt, robot_next_state)
        if robots[robot_id].value > collider.value:
            # 不避障
            # line = Line(array([v_x, v_y]) + dv / 2, n)
            line = Line(array([v_x, v_y]), n)
        elif robots[robot_id].value == collider.value:
            # 承担所有责任
            line = Line(array([v_x, v_y]) + dv / 2, n)
        else:
            # line = Line(array([v_x, v_y]) + dv / 2, n)
            line = Line(array([v_x, v_y]) + dv, n)
        lines.append(line)

    pref_velocity = array([v_x, v_y])
    v = halfplane_optimize(lines, pref_velocity, 2)
    # v_ratio = 6 / np.sqrt(v[0]**2 + v[1]**2)
    # v = v * v_ratio
    # if v is None:
    #     return halfplane_optimize(lines, np.array([0, 0]), 2), lines
    return v, lines

def get_avoidance_velocity(robot, collider, t, dt, robot_next_state):
    """Get the smallest relative change in velocity between agent and collider
    that will get them onto the boundary of each other's velocity obstacle
    (VO), and thus avert collision."""

    # This is a summary of the explanation from the AVO paper.
    #
    # The set of all relative velocities that will cause a collision within
    # time tau is called the velocity obstacle (VO). If the relative velocity
    # is outside of the VO, no collision will happen for at least tau time.
    #
    # The VO for two moving disks is a circularly truncated triangle
    # (spherically truncated cone in 3D), with an imaginary apex at the
    # origin. It can be described by a union of disks:
    #
    # Define an open disk centered at p with radius r:
    # D(p, r) := {q | ||q - p|| < r}        (1)
    #
    # Two disks will collide at time t iff ||x + vt|| < r, where x is the
    # displacement, v is the relative velocity, and r is the sum of their
    # radii.
    #
    # Divide by t:  ||x/t + v|| < r/t,
    # Rearrange: ||v - (-x/t)|| < r/t.
    #
    # By (1), this is a disk D(-x/t, r/t), and it is the set of all velocities
    # that will cause a collision at time t.
    #
    # We can now define the VO for time tau as the union of all such disks
    # D(-x/t, r/t) for 0 < t <= tau.
    #
    # Note that the displacement and radius scale _inversely_ proportionally
    # to t, generating a line of disks of increasing radius starting at -x/t.
    # This is what gives the VO its cone shape. The _closest_ velocity disk is
    # at D(-x/tau, r/tau), and this truncates the VO.

    x = -(array([robot.x, 50 - robot.y]) - array([collider.x, 50 - collider.y]))
    
    v_x = robot_next_state[robot.robot_id][1] * np.cos(robot_next_state[robot.robot_id][0])
    v_y = - robot_next_state[robot.robot_id][1] * np.sin(robot_next_state[robot.robot_id][0])
    c_x = robot_next_state[collider.robot_id][1] * np.cos(robot_next_state[collider.robot_id][0])
    c_y = - robot_next_state[collider.robot_id][1] * np.sin(robot_next_state[collider.robot_id][0])



    v = array([v_x, v_y]) - array([c_x, c_y])
    r1 = cfg.ROBOT_RADIUS if robot.take_thing == 0 else cfg.ROBOT_RADIUS_MAX
    r2 = cfg.ROBOT_RADIUS if collider.take_thing == 0 else cfg.ROBOT_RADIUS_MAX
    r = r1 + r2

    x_len_sq = norm_sq(x)

    if x_len_sq >= r * r:
        # We need to decide whether to project onto the disk truncating the VO
        # or onto the sides.
        #
        # The center of the truncating disk doesn't mark the line between
        # projecting onto the sides or the disk, since the sides are not
        # parallel to the displacement. We need to bring it a bit closer. How
        # much closer can be worked out by similar triangles. It works out
        # that the new point is at x/t cos(theta)^2, where theta is the angle
        # of the aperture (so sin^2(theta) = (r/||x||)^2).
        adjusted_center = x/t * (1 - (r*r)/x_len_sq)

        if dot(v - adjusted_center, adjusted_center) < 0:
            # v lies in the front part of the cone
            # print("front")
            # print("front", adjusted_center, x_len_sq, r, x, t)
            w = v - x/t
            u = normalized(w) * r/t - w
            n = normalized(w)
        else: # v lies in the rest of the cone
            # print("sides")
            # Rotate x in the direction of v, to make it a side of the cone.
            # Then project v onto that, and calculate the difference.
            leg_len = sqrt(x_len_sq - r*r)
            # The sign of the sine determines which side to project on.
            sine = copysign(r, det((v, x)))
            rot = array(
                ((leg_len, sine),
                (-sine, leg_len)))
            rotated_x = rot.dot(x) / x_len_sq
            n = perp(rotated_x)
            if sine < 0:
                # Need to flip the direction of the line to make the
                # half-plane point out of the cone.
                n = -n
            # print("rotated_x=%s" % rotated_x)
            u = rotated_x * dot(v, rotated_x) - v
            # print("u=%s" % u)
    else:
        # We're already intersecting. Pick the closest velocity to our
        # velocity that will get us out of the collision within the next
        # timestep.
        # print("intersecting")
        w = v - x/dt
        u = normalized(w) * r/dt - w
        n = normalized(w)
    return u, n

def norm_sq(x):
    return dot(x, x)

def normalized(x):
    l = norm_sq(x)
    assert l > 0, (x, l)
    return x / sqrt(l)

def dist_sq(a, b):
    return norm_sq(b - a)
