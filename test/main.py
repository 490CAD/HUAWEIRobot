#!/bin/bash
import sys
from robot import Robot
from workbench import WorkBench
from config import CFG
from calcation import *

from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp
from numpy import array, rint, linspace, pi, cos, sin, sqrt

# hyperparameters
cfg = CFG()
log = open("log.txt", "a")
# global needs
high_level_workbench_list = []
useful_workbench_list = []
env_mp, DIS_MP = None, None
workbench_xynum_dist = {}
workbench_ids = 0
workbenchs, robots = [], []
workbench_type_num = [[] for i in range(10)]
simple_job_workbenchs = {}
generate_product = {4:0, 5:0, 6:0}

# log = open('log.txt', 'w')
# newest policy
def get_price_by_targets(free_robots):
    # log.write(f'{generate_product}\n')
    robot_id, target0_id, target1_id, best_val_time = -1, -1, -1, 0.0
    workbench_list = useful_workbench_list
    for robot in free_robots:
        for workbench in workbench_list:
            target0 = workbench
            ava_list, target_workbench_list = [], []
            if workbenchs[target0].is_targeted_flag[0] == 1:
                continue
            robot_dis = cal_point_x_y(robots[robot].x, robots[robot].y, workbenchs[target0].x, workbenchs[target0].y)
            if workbenchs[target0].work_type in [4, 5, 6]:
                target_workbench_list = [7, 9]
            elif workbenchs[target0].work_type == 7:
                target_workbench_list = [8, 9]
            elif workbenchs[target0].work_type == 1:
                target_workbench_list = [4, 5, 9]
                if generate_product[4] - generate_product[5] >= 2:
                    target_workbench_list.pop(0)
                elif generate_product[4] - generate_product[5] <= -2:
                    target_workbench_list.pop(1)
            elif workbenchs[target0].work_type == 2:
                target_workbench_list = [4, 6, 9]
                if generate_product[4] - generate_product[6] >= 2:
                    target_workbench_list.pop(0)
                elif generate_product[4] - generate_product[6] <= -2:
                    target_workbench_list.pop(1)
            elif workbenchs[target0].work_type == 3:
                target_workbench_list = [5, 6, 9]
                if generate_product[5] - generate_product[6] >= 2:
                    target_workbench_list.pop(0)
                elif generate_product[5] - generate_product[6] <= -2:
                    target_workbench_list.pop(1)

            for i in target_workbench_list:
                type_num_list = workbench_type_num[i]
                for j in type_num_list:
                    ava_list.append(j)

            for i in ava_list:
                take_thing = workbenchs[target0].work_type
                if workbenchs[i].work_type in [4, 5, 6, 7]:
                    if  workbenchs[i].is_targeted_flag[take_thing] == 1 or ((1 << take_thing) & workbenchs[i].origin_thing) != 0:
                        continue
                target1 = i
                workbench_dis = DIS_MP[target0][target1]
                all_dis = robot_dis + workbench_dis
                go_time = robot_dis * 50 / 6.0
                all_time = all_dis * 50 / 6.0
                wait_time = workbenchs[target0].remain_time
                if 1 <= workbenchs[target0].work_type <= 3:
                    all_time += 0
                elif wait_time == -1:
                    all_time += cfg.MAX_PENTALIY_VALUE
                elif workbenchs[target0].output == 1 or wait_time <= go_time:
                    all_time += 0
                elif wait_time - go_time > cfg.MAX_WAIT_TIME:
                    all_time += cfg.MAX_PENTALIY_VALUE
                else:
                    all_time += wait_time - go_time
                    
                temp_val = cfg.THING_VALUE[workbenchs[target0].work_type] # * cfg.THING_COEFF[workbenchs[target0].work_type]
                if workbenchs[target1].work_type == 7:
                    if workbenchs[target1].origin_thing == 0:
                        temp_val += cfg.THING_VALUE[workbenchs[target1].work_type] / 15
                    elif ((1 << workbenchs[target0].work_type) | workbenchs[target1].origin_thing) == 112:
                        temp_val += cfg.THING_VALUE[workbenchs[target1].work_type] * 2
                    else:
                        temp_val += cfg.THING_VALUE[workbenchs[target1].work_type] / 10
                elif workbenchs[target1].work_type in [4, 5, 6]:
                    if workbenchs[target1].origin_thing == 0:
                        temp_val += cfg.THING_VALUE[workbenchs[target1].work_type] / 10
                    else:
                        temp_val += cfg.THING_VALUE[workbenchs[target1].work_type] / 5
                    
                elif workbenchs[target1].work_type in [8, 9]:
                    temp_val += cfg.THING_VALUE[workbenchs[target0].work_type] / 5

                temp_val_time = temp_val / all_time

                if temp_val_time > best_val_time:
                    robot_id, target0_id, target1_id = robot, target0, target1
                    best_val_time = temp_val_time
            # target1 = find_nearest_target_sell(workbenchs[workbench].x ,workbenchs[workbench].y, target_workbench_list, workbenchs[workbench].work_type)        
    return robot_id, target0_id, target1_id  

# store something 
def find_nearest_target_sell(x, y, target_workbench_list, take_thing):
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

def get_price_by_time(free_robots):
    robot_id, target0_id, target1_id, best_val_time = -1, -1, -1, 0.0
    workbench_list = useful_workbench_list
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
            target1 = find_nearest_target_sell(workbenchs[workbench].x ,workbenchs[workbench].y, target_workbench_list, workbenchs[workbench].work_type)
            if target1 == -1:
                continue
            all_dis = robot_dis + DIS_MP[target0][target1]
            go_time = robot_dis * 50 / 6.0
            all_time = all_dis * 50 / 6.0
            wait_time = workbenchs[target0].remain_time
            if 1 <= workbenchs[target0].work_type <= 3:
                all_time += 0
            elif wait_time == -1:
                all_time += cfg.MAX_PENTALIY_VALUE
            elif workbenchs[target0].output == 1 or wait_time <= go_time:
                all_time += 0
            elif wait_time - go_time > cfg.MAX_WAIT_TIME:
                all_time += cfg.MAX_PENTALIY_VALUE
            else:
                all_time += workbenchs[target0].remain_time - go_time
            temp_val = cfg.THING_VALUE[workbenchs[workbench].work_type]
            temp_val_time = temp_val / all_time
            if temp_val_time > best_val_time:
                robot_id, target0_id, target1_id = robot, target0, target1
                best_val_time = temp_val_time
    return robot_id, target0_id, target1_id   

# Main
if __name__ == '__main__':
    # input env_map
    env_mp, DIS_MP = read_map(), [[50.0 * 50.0 for j in range(50)] for i in range(50)]
    finish()
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
            
            for robot in range(cfg.ROBOT_NUM):
                robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y = input().split()
                # create a new robot and init it
                robots.append(Robot(robot))
                robots[robot].get_from_frame(robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y)
            # init the distance tables
            
            for workbench_a in range(0, workbench_ids):
                for workbench_b in range(workbench_a + 1, workbench_ids):
                    DIS_MP[workbench_a][workbench_b] = DIS_MP[workbench_b][workbench_a] = cal_point_x_y(workbenchs[workbench_a].x, workbenchs[workbench_a].y, workbenchs[workbench_b].x, workbenchs[workbench_b].y)
            
            for workbench_type in cfg.HIGH_LEVEL_WORKBENCH:
                for workbench in workbench_type_num[workbench_type]:
                    high_level_workbench_list.append(workbench)

            for workbench_type in cfg.USEFUL_WORKBENCH:
                for workbench in workbench_type_num[workbench_type]:
                    useful_workbench_list.append(workbench)
            # log.write(f'{useful_workbench_list}')
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
            
            for robot in range(cfg.ROBOT_NUM):
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
                employ_robot, target0, target1 = get_price_by_targets(free_robots)
                # employ_robot, target0, target1 = get_price_by_time(free_robots)
                robots[employ_robot].target_workbench_ids[0] = target0
                robots[employ_robot].target_workbench_ids[1] = target1
                workbenchs[robots[employ_robot].target_workbench_ids[0]].is_targeted_flag[0] = 1
                workbenchs[robots[employ_robot].target_workbench_ids[1]].is_targeted_flag[workbenchs[robots[employ_robot].target_workbench_ids[0]].work_type] = 1
 
                # employ_robot, target0, target1 = get_price_by_dis(free_robots, free_jobs)
                # employ_robot, target0, target1 = get_job(free_robots, free_jobs)
                # if target1 == -1:
                #     employ_robot, target0, target1 = get_price_simple_by_dis(free_robots)
                #     # employ_robot, target0, target1 = get_simple_job(free_robots)
                #     robots[employ_robot].target_workbench_ids[0] = target0
                #     robots[employ_robot].target_workbench_ids[1] = target1
                #     if robots[employ_robot].target_workbench_ids[1] == -1:
                #         robots[employ_robot].target_workbench_ids[0] = -1
                #     else:
                #         workbenchs[robots[employ_robot].target_workbench_ids[0]].is_targeted_flag[0] = 1
                #         workbenchs[robots[employ_robot].target_workbench_ids[1]].is_targeted_flag[workbenchs[robots[employ_robot].target_workbench_ids[0]].work_type] = 1
                #         simple_job_workbenchs[robots[employ_robot].target_workbench_ids[0]] = 0
                # else:                    
                #     robots[employ_robot].target_workbench_ids[0] = target0
                #     robots[employ_robot].target_workbench_ids[1] = target1
                #     workbenchs[robots[employ_robot].target_workbench_ids[0]].is_targeted_flag[0] = 1
                #     workbenchs[robots[employ_robot].target_workbench_ids[1]].is_targeted_flag[workbenchs[robots[employ_robot].target_workbench_ids[0]].work_type] = 1
                #     free_jobs.pop(target0)
                if employ_robot != -1:
                    free_robots.remove(employ_robot)
        line = sys.stdin.readline()


        # do some operation
        sys.stdout.write('%d\n' % frame_id)
        for robot_id in range(cfg.ROBOT_NUM):
            rotate, forward = None, None
            # if robot_id not in [0]:
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
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        cfg.pid_list[robot_id] = [rotate, forward]

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
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        cfg.pid_list[robot_id] = [rotate, forward]

                elif robots[robot_id].state == 5:
                    # sell and turn 0
                    take_thing = robots[robot_id].take_thing
                    target = robots[robot_id].target_workbench_ids[1]
                    if robots[robot_id].work_space == target and not ((1 << take_thing) & workbenchs[target].origin_thing):
                        sys.stdout.write('sell %d\n' % robot_id)
                        # 将相应原料的卖操作解锁
                        # 1111110
                        if workbenchs[target].work_type == 4 and ((1 << take_thing) | workbenchs[target].origin_thing) == 6:
                            generate_product[4] += 1
                        if workbenchs[target].work_type == 5 and ((1 << take_thing) | workbenchs[target].origin_thing) == 10:
                            generate_product[5] += 1
                        if workbenchs[target].work_type == 6 and ((1 << take_thing) | workbenchs[target].origin_thing) == 12:
                            generate_product[6] += 1
                        

                        target_workbench = robots[robot_id].target_workbench_ids[1]
                        workbenchs[target_workbench].is_targeted_flag[take_thing] = 0

                        robots[robot_id].state = 0
                        robots[robot_id].target_workbench_ids[0] = -1
                        robots[robot_id].target_workbench_ids[1] = -1                      
                    else:
                        robots[robot_id].state = 4

        distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y, robots[robot_id].x + 20, robots[robot_id].y)
                    # direction to target
        direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y, robots[robot_id].x + 20, robots[robot_id].y)

        rotate, forward = robots[3].move_to_target(direction, distance)

        ### 防碰撞检测与预防
        for i, robot in enumerate(robots):
            if i != 3:
                continue
            if cfg.pid_list[i][0] is None:
                continue


            # v, _ = orca(i, robots, cfg.tau, cfg.dt, cfg.pid_list)
            # if cfg.pid_list[i][1] >= 0:
            #     rotate =  math.atan2(-v[1], v[0])  - robot.toward
            #     if rotate > cfg.PI:
            #         rotate += -2*cfg.PI
            #     elif rotate <= -cfg.PI:
            #         rotate += 2*cfg.PI
            #     rotate = rotate / cfg.dt
            #     forward = sqrt(v[0]**2 + v[1]**2)

            # sys.stdout.write('rotate %d %f\n' % (i, rotate))
            sys.stdout.write('forward %d %f\n' % (i, forward))
        ###



        # log.write(f'----------------------------------------------------------------\n')
        finish()
