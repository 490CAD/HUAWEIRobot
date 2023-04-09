import numpy as np
from numpy import array, sqrt, copysign, dot
from numpy.linalg import det
from halfoptimize import halfplane_optimize, Line, perp
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


def orca(robot_id, robots, walls, t, dt, pid_list, mode=0):
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

    # if mode != 3:
    robots[robot_id].value = cfg.THING_VALUE[robots[robot_id].take_thing] * robots[robot_id].time_f * robots[robot_id].crush_f
    for collider in robots[0: robot_id] + robots[robot_id + 1:]:
        # if mode != 3:
        collider.value = cfg.THING_VALUE[collider.take_thing] * collider.time_f * collider.crush_f
        dv, n = get_avoidance_velocity(robots[robot_id], collider, t, dt, robot_next_state)
        if robots[robot_id].value > collider.value:
            # 不避障
            # line = Line(array([v_x, v_y]) + dv / 2, n)
            line = Line(array([v_x, v_y]), n)
        elif robots[robot_id].value == collider.value:
            # 承担一半责任
            line = Line(array([v_x, v_y]) + dv / 2, n)
        else:
            # 承担全部
            line = Line(array([v_x, v_y]) + 2 * dv, n)
        # line = Line(array([v_x, v_y]) + dv / 2, n)
        lines.append(line)
    for wall in walls:
        dv, n = get_avoidance_velocity(robots[robot_id], wall, t, dt, robot_next_state)
        # 承担全部
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

    x = -(array([robot.x, 50 - robot.y]) - array([collider.x, 50 - collider.y]))
    
    v_x = robot_next_state[robot.robot_id][1] * np.cos(robot_next_state[robot.robot_id][0])
    v_y = - robot_next_state[robot.robot_id][1] * np.sin(robot_next_state[robot.robot_id][0])
    if hasattr(collider, 'robot_id'):
        c_x = robot_next_state[collider.robot_id][1] * np.cos(robot_next_state[collider.robot_id][0])
        c_y = - robot_next_state[collider.robot_id][1] * np.sin(robot_next_state[collider.robot_id][0])
    else:
        c_x = 0
        c_y = 0



    v = array([v_x, v_y]) - array([c_x, c_y])
    r1 = cfg.ROBOT_RADIUS if robot.take_thing == 0 else cfg.ROBOT_RADIUS_MAX
    if hasattr(collider, 'take_thing'):
        r2 = cfg.ROBOT_RADIUS if collider.take_thing == 0 else cfg.ROBOT_RADIUS_MAX
    else:
        # r2 = 0.25 * sqrt(2)
        r2 = 0.25
    r = r1 + r2

    x_len_sq = norm_sq(x)

    if x_len_sq >= r * r:
        adjusted_center = x/t * (1 - (r*r)/x_len_sq)

        if dot(v - adjusted_center, adjusted_center) < 0:
            w = v - x/t
            u = normalized(w) * r/t - w
            n = normalized(w)
        else: 
            leg_len = sqrt(x_len_sq - r*r)
            sine = copysign(r, det((v, x)))
            rot = array(
                ((leg_len, sine),
                (-sine, leg_len)))
            rotated_x = rot.dot(x) / x_len_sq
            n = perp(rotated_x)
            if sine < 0:
                n = -n
            # print("rotated_x=%s" % rotated_x)
            u = rotated_x * dot(v, rotated_x) - v
            # print("u=%s" % u)
    else:
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
