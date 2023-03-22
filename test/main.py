#!/bin/bash
import sys
import math
import PID
import random
import copy
# hyperparameters
ROBOT_RADIUS = 0.45
ROBOT_RHO = 20
ROBOT_NUM = 4
MAP_SIZE = 100
PI = math.pi
THING_VALUE = [0, 3000, 3200, 3400, 7100, 7800, 8300, 29000]
HIGH_LEVEL_WORKBENCH = [4, 5, 6, 7]
USEFUL_WORKBENCH = [1, 2, 3, 4, 5, 6, 7]
MAX_WAIT_TIME = 100
MAX_PENTALIY_VALUE = 100000
high_level_workbench_list = []
useful_workbench_list = []
DIS_MP = None
log = open("log.txt","w")
# Robots and Craft Tables
class Robot():
    def __init__(self, id):
        self.work_space = -1
        self.take_thing = 0
        self.time_f = 0.0
        self.crush_f = 0.0
        self.angle_speed = 0.0
        self.line_speed_x, self.line_speed_y = 0.0, 0.0
        self.toward = 0.0
        self.x, self.y = 0.0, 0.0
        self.robot_id = id
        self.task_by_time = 0.0
        # work_frame means start work from this frame
        # state 0 means staying
        # state 1 means walking towards the workbench
        # state 2 means buy the original things
        # state 3 means waiting for the produce
        # state 4 means walking towards the sell things
        # state 5 means sell the things
        # TODO::state 6 means 
        self.work_frame = 1
        self.state = 0
        # next target workbench
        self.target_workbench_ids = [-1, -1]
        self.s_pid=PID.PID(50, 0.001, 1300, 0)
        # 0.8 0.005
        self.w_pid=PID.PID(50, 0.01, 3, 0)

    def get_from_frame(self, work_space, take_thing, time_f, crush_f, angle_speed, line_speed_x, line_speed_y, toward, x, y):
        self.work_space = int(work_space)
        self.take_thing = int(take_thing)
        self.time_f = float(time_f)
        self.crush_f = float(crush_f)
        self.angle_speed = float(angle_speed)
        self.line_speed_x, self.line_speed_y = float(line_speed_x), float(line_speed_y)
        self.toward = float(toward)
        self.x, self.y = float(x), float(y)

    # def find_nearest_target(self, workbench_frame_num, workbenchs, target_workbench_list, take_thing):
    #     target_workbench_distance=300
    #     for workbench_ids in range(workbench_frame_num):
    #         # if (workbenchs[workbench_ids].work_type in target_workbench_list) and workbenchs[workbench_ids].is_targeted_flag == 0:
    #         if ((workbenchs[workbench_ids].work_type in target_workbench_list)) and workbenchs[workbench_ids].is_targeted_flag == 0:
    #             # workbench is full
    #             if ((1 << take_thing) & workbenchs[workbench_ids].origin_thing):
    #                 continue
    #             R_W_distance = cal_point_x_y(self.x, self.y, workbenchs[workbench_ids].x, workbenchs[workbench_ids].y)
    #             if target_workbench_distance > R_W_distance:
    #                 self.target_workbench_ids, target_workbench_distance = workbench_ids, R_W_distance
    def move(self, steering, distance,
                tolerance0=0.001, tolerance1=0.4):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if abs(steering) < tolerance0:
            steering = 0
        if abs(distance) < tolerance1:
            distance = 0
        
        # apply pid
        sys.stdout.write('rotate %d %f\n' % (self.robot_id, steering))

        if distance < 0:
            sys.stdout.write('forward %d %f\n' % (robot_id, -2))
        else:
            speed = min(6, distance)
            sys.stdout.write('forward %d %f\n' % (robot_id, speed))

    def move_to_target(self, direction, distance):
        direction1=direction-self.toward
        if direction1 > PI:
            direction1 += -2*PI
        elif direction1 <= -PI:
            direction1 += 2*PI

        DISTANCE_TOLERATION = 2

        # 左墙面 -->右墙面 -->上墙面 -->下墙面
        if ((self.x <= DISTANCE_TOLERATION) and (self.toward >= PI * 3 / 4 or self.toward <= -PI * 3 / 4)) \
            or ((self.x >= (50 - DISTANCE_TOLERATION)) and (self.toward >= -PI / 4 and self.toward <= PI / 4)) \
            or ((self.y >= (50 - DISTANCE_TOLERATION)) and (self.toward > PI / 4 or self.toward < PI * 3 / 4)) \
            or ((self.y <= DISTANCE_TOLERATION) and (self.toward > -PI * 3 / 4 and self.toward < -PI / 4)):
            if direction1 > PI / 2 or direction1 < -PI / 2:
                distance = - distance
        
        steering = self.w_pid.control(-direction1)
        move_distance = self.s_pid.control(-distance)
        log.write(f'{self.s_pid.accumulated_error} {self.s_pid.previous_error}\n')
        self.move(steering, move_distance)

class WorkBench():
    def __init__(self, id):
        self.work_type = 0
        self.x, self.y = 0.0, 0.0
        self.remain_time = -1
        self.origin_thing =0
        self.output = 0
        self.table_id = id

        # the workbench is targeted by a robot
        # 第0位为买锁，其余第i位为第i钟原料的锁
        self.is_targeted_flag = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    def get_from_frame(self, work_type, x, y, remain_time, origin_thing, output):
        self.work_type = int(work_type)
        self.x, self.y = float(x), float(y)
        self.remain_time = int(remain_time)
        self.origin_thing = int(origin_thing)
        self.output = int(output)


# Calc Functions
def cal_x(row: int):
    return 0.25 + row * 0.50
    
def cal_y(col: int):
    # return 0.25 + col * 0.50
    return 50.00 - (0.25 + col * 0.50)
    
def cal_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y):
    return math.sqrt((origin_x - target_x) ** 2 + (origin_y - target_y) ** 2)

def drt_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y):
    return math.atan2(target_y - origin_y, target_x - origin_x)

def find_nearest_target_sell(x, y, workbenchs, target_workbench_list, workbench_type_num, take_thing):
    ava_list = []
    for i in target_workbench_list:
        type_num_list = workbench_type_num[i]
        for j in type_num_list:
            ava_list.append(j)
    # log.write(f'ava_list: {ava_list}\n')
    target_workbench_ids = -1
    target_workbench_distance = 300
    for i in ava_list:
        if workbenchs[i].work_type in [4, 5, 6, 7]:
            if  workbenchs[i].is_targeted_flag[take_thing] == 1 or ((1 << take_thing) & workbenchs[i].origin_thing) != 0:
                continue
        # if ((1 << take_thing) & workbenchs[workbench_ids].origin_thing) == 0:
        R_W_distance = cal_point_x_y(x, y, workbenchs[i].x, workbenchs[i].y)
        if target_workbench_distance > R_W_distance:
            target_workbench_ids, target_workbench_distance = i, R_W_distance
    return target_workbench_ids

def get_price_simple_by_dis(free_robots, robots, simple_job_workbenchs, workbenchs, workbench_type_num):
    robot_id, target0_id, target1_id = -1, -1, -1
    workbench_list = list(simple_job_workbenchs.keys())
    best_val_dis = 0.0
    for robot in free_robots:
        for workbench in workbench_list:
            target0 = workbench
            if workbenchs[target0].is_targeted_flag[0] == 1:
                continue
            robot_dis = cal_point_x_y(robots[robot].x, robots[robot].y, workbenchs[target0].x, workbenchs[target0].y)
            if workbenchs[target0].work_type == 1:
                target_workbench_list = [4, 5, 9]
            elif workbenchs[target0].work_type == 2:
                target_workbench_list = [4, 6, 9]
            elif workbenchs[target0].work_type == 3:
                target_workbench_list = [5, 6, 9]
            target1 = find_nearest_target_sell(workbenchs[target0].x ,workbenchs[target0].y, workbenchs, target_workbench_list, workbench_type_num, workbenchs[target0].work_type)
            
            if target1 == -1:
                continue
            all_dis = robot_dis + DIS_MP[target0][target1]
            temp_val = THING_VALUE[workbenchs[target0].work_type]
            temp_val_dis = temp_val / all_dis
            if temp_val_dis > best_val_dis:
                robot_id, target0_id, target1_id = robot, target0, target1
                best_val_dis = temp_val_dis

    return robot_id, target0_id, target1_id

def get_simple_job(free_robots, robots, simpe_job_workbenchs, workbenchs, workbench_type_num):
    robot_id, target0, target1 = -1, -1, -1

    job_list = dict(sorted(simpe_job_workbenchs.items(),key=lambda x:x[1],reverse=True))
    for job in job_list.keys():
        target0 = job
        if not workbenchs[target0].is_targeted_flag[0]:
            if workbenchs[target0].work_type == 1:
                # find nearest 4 5 9
                target_workbench_list = [4, 5, 9]
            elif workbenchs[target0].work_type == 2:
                # find nearest 4 6 9
                target_workbench_list = [4, 6, 9]
            elif workbenchs[target0].work_type == 3:
                # find nearest 5 6 9
                target_workbench_list = [5, 6, 9]
            target1 = find_nearest_target_sell(workbenchs[target0].x ,workbenchs[target0].y, workbenchs, target_workbench_list, workbench_type_num, workbenchs[target0].work_type)
        if target1 != -1:
            break
    if target1 == -1:
        return robot_id, target0, target1 
    distance = 300
    for robot in free_robots:
        d = cal_point_x_y(robots[robot].x, robots[robot].y, workbenchs[target0].x, workbenchs[target0].y)
        if distance > d:
            distance = d
            robot_id = robot
    return robot_id, target0, target1

# Input and Output Functions 
def read_map():
    env_mp, row_cnt = [[] for i in range(MAP_SIZE)], 0
    while True:
        line = input()
        if line == "OK":
            break
        for ch in line:
            env_mp[row_cnt].append(ch)
        row_cnt += 1
    return env_mp

def find_free_robot(robots):
    # find workless robot
    free_robot = []
    for ids, robot in enumerate(robots):
        if robot.target_workbench_ids[0] == -1:
            free_robot.append(ids)
    return free_robot

def find_free_job(workbenchs):
    # find available job
    free_job = {}
    for workbench in workbenchs:
        if workbench.output == 1 and workbench.work_type in [7,6,5,4] and workbench.is_targeted_flag[0] == 0:
            free_job[workbench.table_id] = workbench.work_type
        
    return dict(sorted(free_job.items(),key=lambda x:x[1],reverse=True))

def get_price_by_time(free_robots, robots, useful_workbenchs, workbenchs, workbench_type_num):
    robot_id, target0_id, target1_id, best_val_time = -1, -1, -1, 0.0
    workbench_list = useful_workbenchs
    for robot in free_robots:
        for workbench in workbench_list:
            target0 = workbench
            if workbenchs[target0].is_targeted_flag[0] == 1:
                continue
            robot_dis = cal_point_x_y(robots[robot].x, robots[robot].y, workbenchs[target0].x, workbenchs[target0].y)
            if workbenchs[target0].work_type in [4, 5, 6]:
                target_workbench_list = [7, 9]
            elif workbenchs[target0].work_type == 7:
                target_workbench_list = [8, 9]
            elif workbenchs[target0].work_type == 1:
                target_workbench_list = [4, 5, 9]
            elif workbenchs[target0].work_type == 2:
                target_workbench_list = [4, 6, 9]
            elif workbenchs[target0].work_type == 3:
                target_workbench_list = [5, 6, 9]
            target1 = find_nearest_target_sell(workbenchs[workbench].x ,workbenchs[workbench].y, workbenchs, target_workbench_list, workbench_type_num, workbenchs[workbench].work_type)
            if target1 == -1:
                continue
            all_dis = robot_dis + DIS_MP[target0][target1]
            go_time = robot_dis * 50 / 6.0
            all_time = all_dis * 50 / 6.0
            wait_time = workbenchs[target0].remain_time
            if 1 <= workbenchs[target0].work_type <= 3:
                all_time += 0
            elif wait_time == -1:
                all_time += MAX_PENTALIY_VALUE
            elif workbenchs[target0].output == 1 or wait_time <= go_time:
                all_time += 0
            elif wait_time - go_time > MAX_WAIT_TIME:
                all_time += MAX_PENTALIY_VALUE
            else:
                all_time += workbenchs[target0].remain_time - go_time
            temp_val = THING_VALUE[workbenchs[workbench].work_type]
            temp_val_time = temp_val / all_time
            if temp_val_time > best_val_time:
                robot_id, target0_id, target1_id = robot, target0, target1
                best_val_time = temp_val_time
    return robot_id, target0_id, target1_id   


def get_price_by_dis(free_robots, robots, free_jobs, workbenchs, workbench_type_num):
    workbench_list = list(free_jobs.keys())
    robot_id, target0_id, target1_id = -1, -1, -1
    best_val_dis = 0.0
    for robot in free_robots:
        for workbench in workbench_list:
            target0 = workbench
            robot_dis = cal_point_x_y(robots[robot].x, robots[robot].y, workbenchs[target0].x, workbenchs[target0].y)
            if workbenchs[target0].work_type in [4, 5, 6]:
                # find nearest 7, 9
                target_workbench_list = [7, 9]
            elif workbenchs[target0].work_type == 7:
                # find nearest 5 6 9
                target_workbench_list = [8, 9]
            target1 = find_nearest_target_sell(workbenchs[workbench].x ,workbenchs[workbench].y, workbenchs, target_workbench_list, workbench_type_num, workbenchs[workbench].work_type)
            if target1 == -1:
                continue
            all_dis = robot_dis + DIS_MP[target0][target1]
            temp_val = THING_VALUE[workbenchs[workbench].work_type]
            temp_val_dis = temp_val / all_dis
            if temp_val_dis > best_val_dis:
                robot_id, target0_id, target1_id = robot, target0, target1
                best_val_dis = temp_val_dis
    return robot_id, target0_id, target1_id

def get_job(free_robots, robots, free_jobs, workbenchs, workbench_type_num):
    # 按照顺序给任务
    workbench_list = list(free_jobs.keys())
    # log.write(f'{workbench_list}\n')
    robot_id, target0, target1 = -1, -1, -1
    for workbench in workbench_list:
        target0 = workbench
        if workbenchs[target0].work_type in [4, 5, 6]:
            # find nearest 7, 9
            target_workbench_list = [7, 9]
        elif workbenchs[target0].work_type == 7:
            # find nearest 5 6 9
            target_workbench_list = [8, 9]
        target1 = find_nearest_target_sell(workbenchs[target0].x ,workbenchs[target0].y, workbenchs, target_workbench_list, workbench_type_num, workbenchs[target0].work_type)
        if target1 != -1:
            break
    
    if target1 == -1:
        return robot_id, target0, target1

    distance = 300
    for robot in free_robots:
        d = cal_point_x_y(robots[robot].x, robots[robot].y, workbenchs[target0].x, workbenchs[target0].y)
        if distance > d:
            distance = d
            robot_id = robot
    return robot_id, target0, target1
        
def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()
# Main
if __name__ == '__main__':
    # input env_map
    env_mp, DIS_MP = read_map(), [[50.0 * 50.0 for j in range(50)] for i in range(50)]
    finish()

    # init Craft_Table and robots
    workbench_xynum_dist = {}
    workbench_ids = 0
    workbenchs, robots = [], []
    workbench_type_num = [[] for i in range(10)]

    # 生产1，2，3的工作台，value为距上次被访问的时间
    simple_job_workbenchs = {}
    # start working
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        # input every frame
        parts = line.split(' ')
        frame_id, money_frame = int(parts[0]), int(parts[1])
        log.write(f'------------------------------{frame_id} \n')
        if frame_id == 1:
            # 1th frame use init
            workbench_frame_num = int(input())
            for workbench in range(workbench_frame_num):
                workbench_type, workbench_x, workbench_y, workbench_remain, workbench_origin, workbench_output = input().split()
                # create a new craft table
                workbench_xynum_dist[(float(workbench_x), float(workbench_y))] = workbench_ids
                workbenchs.append(WorkBench(workbench_ids))
                workbench_type_num[int(workbench_type)].append(workbench_ids)
                if int(workbench_type) in [1, 2, 3]:
                    simple_job_workbenchs[int(workbench_ids)] = 0
                # init it 
                workbenchs[workbench_ids].get_from_frame(workbench_type, workbench_x, workbench_y, workbench_remain, workbench_origin, workbench_output)
                workbench_ids += 1
            
            for robot in range(ROBOT_NUM):
                robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y = input().split()
                # create a new robot and init it
                robots.append(Robot(robot))
                robots[robot].get_from_frame(robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y)
            # init the distance tables
            
            for workbench_a in range(0, workbench_ids):
                for workbench_b in range(workbench_a + 1, workbench_ids):
                    DIS_MP[workbench_a][workbench_b] = DIS_MP[workbench_b][workbench_a] = cal_point_x_y(workbenchs[workbench_a].x, workbenchs[workbench_a].y, workbenchs[workbench_b].x, workbenchs[workbench_b].y)
            
            for workbench_type in HIGH_LEVEL_WORKBENCH:
                for workbench in workbench_type_num[workbench_type]:
                    high_level_workbench_list.append(workbench)

            for workbench_type in USEFUL_WORKBENCH:
                for workbench in workbench_type_num[workbench_type]:
                    useful_workbench_list.append(workbench)
            # gogogo
           # for robot in range(ROBOT_NUM):
               # sys.stdout.write('forward %d %f\n' % (robot, 2.0))
        else:
            # update
            workbench_frame_num = int(input())
            for workbench in range(workbench_frame_num):
                workbench_type, workbench_x, workbench_y, workbench_remain, workbench_origin, workbench_output = input().split()
                # create a new craft table
                workbench_id = workbench_xynum_dist[(float(workbench_x), float(workbench_y))]
                # update it
                workbenchs[workbench_id].get_from_frame(workbench_type, workbench_x, workbench_y, workbench_remain, workbench_origin, workbench_output)
            
            for robot in range(ROBOT_NUM):
                robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y = input().split()
                # update the robot state
                robots[robot].get_from_frame(robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y)
            
            ###
            # 等待时间加1
            for workbench in simple_job_workbenchs.keys():
                simple_job_workbenchs[workbench] += 1
            # 分配任务
            free_robots = find_free_robot(robots)
            free_jobs = find_free_job(workbenchs)
            # for i in range(ROBOT_NUM):
            #     log.write(f'{i} {robots[i].target_workbench_ids[0]} {robots[i].target_workbench_ids[1]}\n')
            # log.write(f'{free_robots} \n')
            # log.write(f'{free_jobs}\n')

            for i in range(len(free_robots)):
                employ_robot, target0, target1 = get_price_by_time(free_robots, robots, useful_workbench_list,workbenchs, workbench_type_num)
                # employ_robot, target0, target1 = get_price_by_dis(free_robots, robots, free_jobs, workbenchs, workbench_type_num)
                # employ_robot, target0, target1 = get_job(free_robots, robots, free_jobs, workbenchs, workbench_type_num)
                if target1 == -1:
                    employ_robot, target0, target1 = get_price_simple_by_dis(free_robots, robots, simple_job_workbenchs, workbenchs, workbench_type_num)
                    # employ_robot, target0, target1 = get_simple_job(free_robots, robots, simple_job_workbenchs, workbenchs, workbench_type_num)
                    robots[employ_robot].target_workbench_ids[0] = target0
                    robots[employ_robot].target_workbench_ids[1] = target1
                    if robots[employ_robot].target_workbench_ids[1] == -1:
                        robots[employ_robot].target_workbench_ids[0] = -1
                    else:
                        workbenchs[robots[employ_robot].target_workbench_ids[0]].is_targeted_flag[0] = 1
                        workbenchs[robots[employ_robot].target_workbench_ids[1]].is_targeted_flag[workbenchs[robots[employ_robot].target_workbench_ids[0]].work_type] = 1
                        simple_job_workbenchs[robots[employ_robot].target_workbench_ids[0]] = 0
                else:                    
                    robots[employ_robot].target_workbench_ids[0] = target0
                    robots[employ_robot].target_workbench_ids[1] = target1
                    workbenchs[robots[employ_robot].target_workbench_ids[0]].is_targeted_flag[0] = 1
                    workbenchs[robots[employ_robot].target_workbench_ids[1]].is_targeted_flag[workbenchs[robots[employ_robot].target_workbench_ids[0]].work_type] = 1
                    # free_jobs.pop(target0)
                if employ_robot != -1:
                    free_robots.remove(employ_robot)
        line = sys.stdin.readline()

        # do some operation
        sys.stdout.write('%d\n' % frame_id)
        for robot_id in range(ROBOT_NUM):
            # if robot_id not in [2]:
            #     continue
            if robots[robot_id].target_workbench_ids[0] == -1:
                continue
            if robots[robot_id].work_frame <= frame_id:
                if robots[robot_id].state == 0:
                    # find nearest 1 2 3
                    # sys.stdout.write('forward %d %f\n' % (robot_id, 0))
                    robots[robot_id].state = 1
                elif robots[robot_id].state == 1:
                    # calc the speed and go
                    # distance to target
                    distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y, workbenchs[robots[robot_id].target_workbench_ids[0]].x, workbenchs[robots[robot_id].target_workbench_ids[0]].y)
                    # direction to target
                    direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y, workbenchs[robots[robot_id].target_workbench_ids[0]].x, workbenchs[robots[robot_id].target_workbench_ids[0]].y)
                    # reach the target workbench
                    if robots[robot_id].work_space == robots[robot_id].target_workbench_ids[0]:
                        robots[robot_id].s_pid.clear()
                        robots[robot_id].w_pid.clear()
                        sys.stdout.write('forward %d %f\n' % (robot_id, 0))
                        robots[robot_id].state = 2
                    else:
                        robots[robot_id].move_to_target(direction, distance)

                elif robots[robot_id].state == 2:
                    # buy
                    if workbenchs[robots[robot_id].target_workbench_ids[0]].output == 1 and robots[robot_id].work_space == robots[robot_id].target_workbench_ids[0]:
                        if DIS_MP[robots[robot_id].target_workbench_ids[0]][robots[robot_id].target_workbench_ids[1]] / 6 > (9000 - frame_id) / 50 - 1:
                            # go to 8,9
                            continue
                        sys.stdout.write('buy %d\n' % robot_id)
                        workbenchs[robots[robot_id].target_workbench_ids[0]].is_targeted_flag[0] = 0
                        robots[robot_id].state = 3
                    else:
                        robots[robot_id].state = 1
                    
                elif robots[robot_id].state == 3:
                    robots[robot_id].state = 4

                elif robots[robot_id].state == 4:
                    # calc the speed and go
                    # distance to target
                    distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y, workbenchs[robots[robot_id].target_workbench_ids[1]].x, workbenchs[robots[robot_id].target_workbench_ids[1]].y)
                    # direction to target
                    direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y, workbenchs[robots[robot_id].target_workbench_ids[1]].x, workbenchs[robots[robot_id].target_workbench_ids[1]].y)
                    # reach the target workbench
                    if robots[robot_id].work_space == robots[robot_id].target_workbench_ids[1]:
                        robots[robot_id].s_pid.clear()
                        robots[robot_id].w_pid.clear()
                        sys.stdout.write('forward %d %f\n' % (robot_id, 0))
                        robots[robot_id].state = 5
                    else:
                        robots[robot_id].move_to_target(direction, distance)

                elif robots[robot_id].state == 5:
                    # sell and turn 0
                    take_thing = robots[robot_id].take_thing
                    if robots[robot_id].work_space == robots[robot_id].target_workbench_ids[1] and not ((1 << robots[robot_id].take_thing) & workbenchs[robots[robot_id].target_workbench_ids[1]].origin_thing):
                        sys.stdout.write('sell %d\n' % robot_id)
                        # 将相应原料的卖操作解锁

                        target_workbench = robots[robot_id].target_workbench_ids[1]
                        workbenchs[target_workbench].is_targeted_flag[take_thing] = 0


                        robots[robot_id].state = 0
                        robots[robot_id].target_workbench_ids[0] = -1
                        robots[robot_id].target_workbench_ids[1] = -1                      
                    else:
                        robots[robot_id].state = 4
        finish()
        # log.write(f'~~~~~~~~~~~~~~\n')
        # for i in workbenchs:
        #     log.write(f"{i.table_id} {i.is_targeted_flag}\n")
        # log.write(f"------------------------\n")
