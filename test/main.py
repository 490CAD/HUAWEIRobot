'''
    TODO:
        main.py 353行 防碰撞 开不开 [倒车防碰撞开不开]
        robot.py 88行 倒车 / 墙面倒车
        pyroca.py 71行 承担责任 [86行 检测碰撞]

        robot.py PID超参数
        config.py 检测碰撞超参数

        策略迭代和优化
            3 固定路径 1 打野
            对于7 考虑3段任务
            针对地图分类讨论
'''
#!/bin/bash
import functools
import sys
import functools
from robot import Robot
from workbench import WorkBench
from config import CFG
from calcation import *
# import argparse
from contraception import Agent, get_avoidance_velocity, orca, normalized, perp
from numpy import array, rint, linspace, pi, cos, sin, sqrt
import numpy as np
import queue
from collections import deque
from wall import Wall

import time

# hyperparameters
cfg = CFG()
# log = open("log.txt", "w")
# global needs

# map1 43
# map2 25
# map3 50
# map4 18
"""
    useful_workbench_list: 类型为1234567的工作台
    env_mp: 地图
    DIS_MP: DIS_MP[a][b]表示a号工作台与b号工作台之间的距离
    workbench_ids: 工作台总数量
    workbenchs: 工作台列表
    workbench_type_num: workbench_type_num[i]表示工作台类型为i的所有工作台
    workbench_minest_sell: workbench_minest_sell[i]表示对于i号工作台他能卖的最近的工作台编号
    generate_product: 全场生产4 5 6的数量

    calcation方法:
        cal_point_x_y(x1, y1, x2, y2): 算两点距离
        drt_point_x_y(x1, y1, x2, y2): 算两点角度
        find_free_robot(robots) 
        find_free_job(workbenchs)顾名思义
        add_more_times_all(workbench, wait_time, go_time): 将尚未生产出来的物品纳入考虑
        choose_target_workbench_list(generate_product, origin_workbench_work_type, choose_mode=1): 顾名思义, 1是默认状态, 2是特殊状态 将会根据456的生成情况删除相应的4 或5或6
        get_ava_list(target_workbench_list, workbench_type_num): 根据target_workbench_list得到ava_list
"""
useful_workbench_list = []
env_mp, DIS_MP, DIS_MP_nothing = None, [], []
workbench_ids = 0
useful_workbench_cnt = 0
workbenchs, robots, walls = [], [], []
workbench_type_num = [[] for i in range(10)]
workbench_minest_sell = []
generate_product = {4:0, 5:0, 6:0}
workbench_mode = 0
# new_env_mp = [['' for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
# new_env_mp = [['' for i in range(cfg.MAP_SIZE_2)] for j in range(cfg.MAP_SIZE_2)]
# mask_env_mp = [[0 for i in range(cfg.MAP_SIZE_2)] for j in range(cfg.MAP_SIZE_2)]
map_limit = [0, 0, 0, 0]
DIS_MP_dict, DIS_MP_nothing_dict = {}, {}

workbench_allocate_list = []
finished_list, task_list, waiting_list, generate_list = [], [], [], []
task_pos_list = [0 for i in range(8)]
# 
refuse_workbench_dist = {}
workbench_taking_mp, workbench_nothing_mp = [], []
index_taking_mp, index_nothing_mp = [], []
robot_taking_mp, robot_index_taking_mp = [[] for i in range(cfg.ROBOT_NUM)], [[] for i in range(cfg.ROBOT_NUM)]
path_better_taking_mp, path_better_nothing_mp = [], []
wall_list = [[] for i in range(4)]
battle_time = [0, 0, 0, 0]
back_distannce = [0, 0, 0, 0]
back_waittime = [0, 0, 0, 0]
astar_workbench_taking_mp, astar_workbench_nothing_mp = [], []
astar_robot_taking_mp, astar_robot_nothing_mp = [], []
task_list, task_cache = [], []
# # Arguments for up_down_policy_sxw function
# # 7 and 654
# task_allocate_list = [[], []]
# # 7 and 654 and 321
# task_completed_list = [[], [], []]

GRA_MP = None
LOCK_MAP = None
mask_path_mp = [[[0 for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)] for i in range(4)]
def update_path_lock(path_list, robot_id):
    global mask_path_mp
    mask_path_mp = np.zeros_like(mask_path_mp)
    for i in range(4):
        if robot_id != i:
            for point in path_list:
                # log.write(f"i point0 point1 is {i}, {point[0]}, {point[1]}\n")
                mask_path_mp[i][anti_cal_y(point[1])][anti_cal_x(point[0])] = 1

def get_price_by_targets_half(free_robots, work_mode, frame_id):
    """
        robot_id -> 执行任务的机器人, target0_id -> 去买的工作台, target1_id ->去卖的工作台
        best_val_time -> max(盈利 / 时间(robot->target0->target1))
        workbench_list -> 工作台类型为1-7的工作台
        robot_dis -> robot到target0的距离
        target_workbench_list -> 可选的target1的工作台类型列表
        ava_list -> 可选的target1工作台列表
        all_dis -> robot_dis + target0到target1的距离
        go_time -> robot_dis花费时间
        wait_time -> 工作台target0生产物品所需要的剩余时间
        all_time -> 整个过程的时间
    """
    global workbench_ids
    global workbench_mode, useful_workbench_cnt
    global robot_taking_mp, robot_index_taking_mp
    global workbench_taking_mp, index_taking_mp

    robot_id, target0_id, target1_id, best_val_time = -1, -1, -1, 0.0
    workbench_list = useful_workbench_list

    for id in free_robots:
        robot = robots[id]

        for target0 in workbench_list:
            target0_workbench = workbenchs[target0]

            if useful_workbench_cnt > 8:
                if robot.now_suppose_work_space == -1:
                    if robot_taking_mp[id].get((target0_workbench.half_x, target0_workbench.half_y)) is None:
                        continue
                else:
                    if workbench_taking_mp[robot.now_suppose_work_space].get((target0_workbench.half_x, target0_workbench.half_y)) is None:
                        continue
                    
            else:
                if robot.now_suppose_work_space == -1:
                    if robot_taking_mp[id].get((target0_workbench.half_x, target0_workbench.half_y)) is None:
                        continue
                else:
                    if workbench_nothing_mp[robot.now_suppose_work_space].get((target0_workbench.half_x, target0_workbench.half_y)) is None:
                        continue
                    
            if target0_workbench.is_targeted_flag[0] == 1 or (target0_workbench.output != 1 and target0_workbench.work_type in cfg.HIGH_LEVEL_WORKBENCH and target0_workbench.remain_time == -1):
                continue
            
            
            target_workbench_list = choose_target_workbench_list(generate_product, target0_workbench.work_type, work_mode)
            ava_list = get_ava_list(target_workbench_list, workbench_type_num)

            for target1 in ava_list:
                target1_workbench = workbenchs[target1]

                if workbench_taking_mp[target0].get((target1_workbench.half_x, target1_workbench.half_y)) is None:
                    continue
                if workbench_mode == 2:
                    if id == 0:
                        if target0 not in [11, 12, 18] or target1 not in [11, 12, 18, 9]:
                            continue
                    if id == 1:
                        if target0 not in [13, 14, 19] or target1 not in [13, 14, 19, 9]:
                            continue
                    if id == 2:
                        if target0 not in [16, 20, 22] or target1 not in [16, 20, 22, 9]:
                            continue
                    if id == 3:
                        if target0 not in [7, 9] or target1 not in [7, 9]:
                            continue

                # if id == 2 and target0 == 5 and target1 == 3:
                #     log.write('!: \n')

                if target1_workbench.work_type in  cfg.HIGH_LEVEL_WORKBENCH:
                    if  target1_workbench.is_targeted_flag[target0_workbench.work_type] == 1 or ((1 << target0_workbench.work_type) & target1_workbench.origin_thing) != 0:
                        continue
                
                # if id == 2:
                #     log.write(f"====\n")
                #     log.write(f"{id, target0, target1}\n")
                #     log.write(f"====\n")

                target0_target1_dis = DIS_MP_dict[(target0, target1)]

                if useful_workbench_cnt > 8:
                    if robot.now_suppose_work_space == -1:
                        robot_target0_dis = robot_index_taking_mp[id][(target0_workbench.half_x, target0_workbench.half_y)]
                    else:
                        robot_target0_dis = DIS_MP_dict[(robot.now_suppose_work_space, target0)]
                else:
                    if robot.now_suppose_work_space == -1:
                        robot_target0_dis = robot_index_taking_mp[id][(target0_workbench.half_x, target0_workbench.half_y)]
                    else:
                        robot_target0_dis = DIS_MP_nothing_dict[(robot.now_suppose_work_space, target0)]

                robot_turn_dis = abs(drt_point_x_y(robot.x, robot.y, target0_workbench.x, target0_workbench.y, work_mode) - robot.toward)

                turn_time = robot_turn_dis * 50 / cfg.PI / 6

                all_dis = robot_target0_dis + target0_target1_dis

                wait_time = target0_workbench.remain_time
                robot_target0_time = robot_target0_dis * 50 / 6
                all_time = all_dis * 50 / 6 + add_more_times_all(target0_workbench, wait_time, robot_target0_time + turn_time, workbench_mode) +turn_time
                temp_val = cfg.THING_VALUE[target0_workbench.work_type]
                temp_val_time = temp_val / all_time
                # next_time = 0
                # if target1_workbench.work_type in cfg.HIGH_LEVEL_WORKBENCH:
                #     if workbench_minest_sell[target1][0] == -1:

                #         next_time = DIS_MP_dict[(target1, workbench_minest_sell[target1][1])] * 50 / 6
                #     elif workbench_minest_sell[target1][1] == -1:
                #         next_time = DIS_MP_dict[(target1, workbench_minest_sell[target1][0])] * 50 / 6
                #     else:
                #         next_time = min(DIS_MP_dict[(target1, workbench_minest_sell[target1][1])], DIS_MP_dict[(target1, workbench_minest_sell[target1][0])]) * 50 / 6

                # if target1_workbench.work_type == 7:
                #     if target1_workbench.origin_thing == 0:
                #         next_time = (next_time + 1000) * 3
                #     elif ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 112:
                #         next_time = (next_time + 1000)
                #     else:
                #         next_time = (next_time + 1000) * 2
                # elif target1_workbench.work_type in [4, 5, 6]:
                #     if target1_workbench.origin_thing == 0:
                #         next_time = (next_time + 500) * 2
                #     else:
                #         next_time = (next_time + 500)
                # if target1_workbench.work_type not in [8, 9] and all_time <= cfg.MAX_PENTALIY_VALUE:
                #     if (target1_workbench.work_type == 7 and ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 112):
                #         temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] # / next_time
                #     if (target1_workbench.work_type == 4 and ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 6):
                #         temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] / next_time
                #     if (target1_workbench.work_type == 5 and ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 10):
                #         temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] / next_time
                #     if (target1_workbench.work_type == 6 and (((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 12)):
                #         temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] / next_time
                if temp_val_time > best_val_time:
                    robot_id, target0_id, target1_id = id, target0, target1
                    best_val_time = temp_val_time
    robots[robot_id].value = best_val_time
    return robot_id, target0_id, target1_id  


def map_init():
    global workbench_ids
    global workbench_mode, workbench_taking_mp, index_taking_mp, workbench_nothing_mp, index_nothing_mp
    global new_env_mp, env_mp, mask_env_mp, path_better_taking_mp, path_better_nothing_mp, DIS_MP
    global astar_workbench_taking_mp, astar_workbench_nothing_mp
    global map_limit, DIS_MP_nothing
    robot_ids = 0
    wall_ids = 0
    for row in range(cfg.MAP_SIZE):
        for col in range(cfg.MAP_SIZE):
            # new_env_mp[row * 2][col * 2] = new_env_mp[row * 2 + 1][col * 2] = new_env_mp[row * 2][col * 2 + 1] = new_env_mp[row * 2 + 1][col * 2 + 1] = env_mp[row][col]
            if env_mp[row][col] == '.':
                continue
            elif '1' <= env_mp[row][col] <= '9':
                workbench_type = int(env_mp[row][col])
                workbenchs.append(WorkBench(workbench_ids))
                workbench_type_num[workbench_type].append(workbench_ids)
                workbench_minest_sell.append([])
                workbenchs[workbench_ids].work_type = workbench_type
                workbenchs[workbench_ids].x, workbenchs[workbench_ids].y = cal_x(col), cal_y(row)
                workbenchs[workbench_ids].anti_x, workbenchs[workbench_ids].anti_y = row * 2, col * 2
                workbenchs[workbench_ids].half_x, workbenchs[workbench_ids].half_y = float(row), float(col)
                if workbenchs[workbench_ids].work_type in cfg.USEFUL_WORKBENCH:
                    useful_workbench_list.append(workbench_ids)
                workbench_ids += 1
            elif env_mp[row][col] == 'A':
                robots.append(Robot(robot_ids))
                robots[robot_ids].x, robots[robot_ids].y = cal_x(col), cal_y(row)
                robots[robot_ids].anti_x, robots[robot_ids].anti_y = row * 2, col * 2
                robots[robot_ids].half_x, robots[robot_ids].half_y = float(row), float(col)

                robot_ids += 1
            elif env_mp[row][col] == '#':
                wall = Wall(wall_ids, cal_x(col), cal_y(row))
                walls.append(wall)
                wall_ids += 1

    for row in range(cfg.MAP_SIZE):
        flag = 0
        for col in range(cfg.MAP_SIZE):
            if env_mp[row][col] != '.':
                flag = 1
                break
        if flag == 1:
            # log.write(f"{row} ")
            map_limit[0] = max(0, row - 5)
            break
    for row in range(cfg.MAP_SIZE - 1, 0, -1):
        flag = 0
        for col in range(cfg.MAP_SIZE):
            if env_mp[row][col] != '.':
                flag = 1
                break
        if flag == 1:
            # log.write(f"{row} ")
            map_limit[1] = min(cfg.MAP_SIZE - 1, row + 5)
            break
    for col in range(cfg.MAP_SIZE):
        flag = 0
        for row in range(cfg.MAP_SIZE):
            if env_mp[row][col] != '.':
                flag = 1
                break
        if flag == 1:
            # log.write(f"{col} ")
            map_limit[2] = max(0, col - 5)
            break
    for col in range(cfg.MAP_SIZE - 1, 0, -1):
        flag = 0
        for row in range(cfg.MAP_SIZE):
            if env_mp[row][col] != '.':
                flag = 1
                break
        if flag == 1:
            # log.write(f"{col} \n")
            map_limit[3] = min(cfg.MAP_SIZE - 1, col + 5)
            break
    # log.write(f"{map_limit}\n")

    workbench_taking_mp, index_taking_mp = [[] for i in range(workbench_ids)],  [[] for i in range(workbench_ids)]
    workbench_nothing_mp, index_nothing_mp = [[] for i in range(workbench_ids)], [[] for i in range(workbench_ids)]
    path_better_taking_mp = [[None for i in range(workbench_ids)] for j in range(workbench_ids)]
    path_better_nothing_mp = [[None for i in range(workbench_ids)] for j in range(workbench_ids)]

    astar_workbench_taking_mp = [[None for i in range(workbench_ids)] for j in range(workbench_ids)]
    astar_workbench_nothing_mp = [[None for i in range(workbench_ids)] for j in range(workbench_ids)]

    DIS_MP = [[40000 for i in range(workbench_ids)] for j in range(workbench_ids)]
    DIS_MP_nothing = [[40000 for i in range(workbench_ids)] for j in range(workbench_ids)]

    if workbench_ids == 9:
        workbench_mode = 1
    elif workbench_ids == 27:
        workbench_mode = 2
        # t_cnt = 0

        # for row in range(cfg.MAP_SIZE):
        #     for col in range(cfg.MAP_SIZE):
        #         new_env_mp[row][col] = env_mp[row][col]

        # for row in range(cfg.MAP_SIZE):
        #     for col in range(cfg.MAP_SIZE):
        #         if env_mp[row][col] == '#':
        #             t_cnt += 1
        #             if t_cnt == 11 or t_cnt >= 19:
        #                 env_mp[row - 1][col] = env_mp[row + 1][col] = env_mp[row][col - 1] = env_mp[row][col + 1] = 'z'
        #                 env_mp[row - 1][col - 1] = env_mp[row - 1][col + 1] = env_mp[row + 1][col - 1] = env_mp[row + 1][col + 1] = 'z'
        
        # for row in range(cfg.MAP_SIZE):
        #     for col in range(cfg.MAP_SIZE):
        #         if env_mp[row][col] == 'z':
        #             env_mp[row][col] = '#'
    
    # if workbench_ids == 50:
    #     workbench_mode = 3
    #     cfg.tau = 72 * cfg.dt
    # elif workbench_ids == 43:
    #     workbench_mode = 1
    #     cfg.tau = 123 * cfg.dt
    # elif workbench_ids == 25:
    #     workbench_mode = 2
    #     # cfg.tau = 37 * cfg.dt
    # elif workbench_ids == 18:
    #     workbench_mode = 4
    #     # 33 29 39_61 42
    #     cfg.tau = 33 * cfg.dt
                        
    # if workbench_mode == 3:
    #     workbench_type_num[4] = sorted(workbench_type_num[4], key=functools.cmp_to_key(map3cmp))
    #     workbench_type_num[5] = sorted(workbench_type_num[5], key=functools.cmp_to_key(map3cmp))
    #     workbench_type_num[6] = sorted(workbench_type_num[6], key=functools.cmp_to_key(map3cmp))
        # workbench_type_num[6] = workbench_type_num[6][0:2]
    # if workbench_mode == 1:
    #     workbench_type_num[7] = sorted(workbench_type_num[7], key=functools.cmp_to_key(map1cmp))
    # if workbench_mode == 4:
    #     workbench_type_num[4] = sorted(workbench_type_num[4], key=functools.cmp_to_key(map4cmp))
    #     workbench_type_num[5] = sorted(workbench_type_num[5], key=functools.cmp_to_key(map4cmp))
    #     workbench_type_num[6] = sorted(workbench_type_num[6], key=functools.cmp_to_key(map4cmp))

# def map4cmp(x, y):
#     x_dis = cal_point_x_y(workbenchs[x].x, workbenchs[x].y, workbenchs[workbench_type_num[7][0]].x, workbenchs[workbench_type_num[7][0]].y)
#     y_dis = cal_point_x_y(workbenchs[y].x, workbenchs[y].y, workbenchs[workbench_type_num[7][0]].x, workbenchs[workbench_type_num[7][0]].y)
#     if x_dis < y_dis:
#         return -1
#     if x_dis > y_dis:
#         return 1
#     return 0

# def map3cmp(x, y):
#     x_dis = cal_point_x_y(workbenchs[x].x, workbenchs[x].y, workbenchs[workbench_type_num[9][0]].x, workbenchs[workbench_type_num[9][0]].y)
#     y_dis = cal_point_x_y(workbenchs[y].x, workbenchs[y].y, workbenchs[workbench_type_num[9][0]].x, workbenchs[workbench_type_num[9][0]].y)
#     if x_dis < y_dis:
#         return -1
#     if x_dis > y_dis:
#         return 1
#     return 0



# def map1cmp(x, y):
#     x_dis = cal_point_x_y(workbenchs[x].x, workbenchs[x].y, workbenchs[workbench_type_num[8][0]].x, workbenchs[workbench_type_num[8][0]].y)
#     y_dis = cal_point_x_y(workbenchs[y].x, workbenchs[y].y, workbenchs[workbench_type_num[8][0]].x, workbenchs[workbench_type_num[8][0]].y)
#     if x_dis < y_dis:
#         return -1
#     if x_dis > y_dis:
#         return 1
#     return 0

def calculate_path_length(robot):
    # target_list 机器人规划路径
    dis = 0
    for idx, path in enumerate([robot.move_list_target0, robot.move_list_target1]):
        if(path[0] == (-1, -1)):
            continue
        elif(path[-1] == (-1, -1)):
            if idx == 0:
                dis += DIS_MP_dict[(robot.now_suppose_work_space, robot.target_workbench_ids[0])] if robot.now_suppose_work_space != robot.target_workbench_ids[0] else 0
            else:
                dis += DIS_MP_dict[(robot.target_workbench_ids[0], robot.target_workbench_ids[1])] if robot.target_workbench_ids[0] != robot.target_workbench_ids[1] else 0
        else:
            for idx2, point in enumerate(path):
                if idx2 == 0:
                    dis += cal_point_x_y(robot.x, robot.y, point[0], point[1])
                elif point == (-1, -1):
                    break
                else:
                    dis += cal_point_x_y(path[idx2-1][0], path[idx2-1][1], point[0], point[1])
    return dis

def workbench_cmp2(x, y):
    if workbenchs[x].work_type < workbenchs[y].work_type:
        return -1
    if workbenchs[x].work_type > workbenchs[y].work_type:
        return 1
    return 0        

def find_nearest_workbench(ed_id, st_id_list):
    min_dis = 1000000
    min_id = -1
    if len(st_id_list) == 0:
        return min_id
    for st in st_id_list:
        if(min_dis > DIS_MP_dict[(st, ed_id)] and can_thing_put_in(st, ed_id, 1)):
            min_dis = DIS_MP_dict[(st, ed_id)]
            min_id = st
        # if(min_dis > DIS_MP[st][ed_id] and can_thing_put_in(st, ed_id, 1)):
        #     min_dis = DIS_MP[st][ed_id]
        #     min_id = st
    return min_id

def can_thing_put_in(st_id, ed_id, put_in_mode = 0):
    # put_in_mode 
    # 0 代表是原始版本 即如果起点成品槽没物体 就为0
    # 1 代表是弱化版本 即起点成品槽没物体也可以为1 只看锁的情况 用于update_GRA_MP和分配任务的DFS
    st_worktype = workbenchs[st_id].work_type
    # log.write(f"workbenchs len is {len(workbenchs)}.\n")
    # log.write(f"ed_id is {ed_id}.\n")
    ed_worktype = workbenchs[ed_id].work_type
    if DIS_MP_dict[(st_id, ed_id)] >= 40000:
        return 0
    if ed_worktype in [4]:
        if st_worktype in [1, 2] and workbenchs[ed_id].is_targeted_flag[st_worktype] == 0 and ((1 << st_worktype) & workbenchs[ed_id].origin_thing) == 0:
            return 1
        else:
            return 0
    elif ed_worktype in [5]:
        if st_worktype in [1, 3] and workbenchs[ed_id].is_targeted_flag[st_worktype] == 0 and ((1 << st_worktype) & workbenchs[ed_id].origin_thing) == 0:
            return 1
        else:
            return 0
    elif ed_worktype in [6]:
        if st_worktype in [2, 3] and workbenchs[ed_id].is_targeted_flag[st_worktype] == 0 and ((1 << st_worktype) & workbenchs[ed_id].origin_thing) == 0:
            return 1
        else:
            return 0
    elif ed_worktype in [7]:
        if st_worktype in [4, 5, 6] and (workbenchs[st_id].output == 1 or put_in_mode == 1) and workbenchs[st_id].is_targeted_flag[0] == 0 and workbenchs[ed_id].is_targeted_flag[st_worktype] == 0 and ((1 << st_worktype) & workbenchs[ed_id].origin_thing) == 0:
            return 1
        else:
            return 0
    elif ed_worktype in [8]:
        if st_worktype in [7] and (workbenchs[st_id].output == 1 or put_in_mode == 1) and workbenchs[st_id].is_targeted_flag[0] == 0 and workbenchs[ed_id].is_targeted_flag[st_worktype] == 0 and ((1 << st_worktype) & workbenchs[ed_id].origin_thing) == 0:
            return 1
        else:
            return 0
    elif ed_worktype in [9]:
        if st_worktype in [1, 2, 3]:
            return 1
        elif st_worktype in [4, 5, 6, 7]:
            if (workbenchs[st_id].output == 1 or put_in_mode == 1) and workbenchs[st_id].is_targeted_flag[0] == 0:
                return 1
            else:
                return 0
        else:
            return 0
        
def workbench_cmp3(x, y):
    if workbenchs[x].work_type < workbenchs[y].work_type:
        return 1
    elif workbenchs[x].work_type > workbenchs[y].work_type:
        return -1
    else:
        dis_x, dis_y = 0, 0
        for i in np.where(GRA_MP[x] == 1)[0]:
            dis_x += DIS_MP_dict[(x, i)]
        for i in np.where(GRA_MP[y] == 1)[0]:
            dis_y += DIS_MP_dict[(y, i)]
        if dis_x < dis_y:
            return -1
        else:
            return 1

# 已知一个工作台，找离它最近的一个机器人
def find_nearest_robot_workbench(robot_ids, workbench_id):
    min_dis = 40000
    min_id = -1
    for robot_id in robot_ids:
        if workbench_mode == 1 and robot_id != 3:
            continue
        if robot_taking_mp[robot_id].get((workbenchs[workbench_id].half_x, workbenchs[workbench_id].half_y)) is None:
            continue
        if robots[robot_id].now_suppose_work_space == -1:
            dis = robot_index_taking_mp[robot_id][(workbenchs[workbench_id].half_x, workbenchs[workbench_id].half_y)] * 0.5
        elif robots[robot_id].target_workbench_ids == [-1, -1]:
            dis = DIS_MP_dict[(robots[robot_id].now_suppose_work_space, workbench_id)] if robots[robot_id].now_suppose_work_space != workbench_id else 0
        else:
            dis = calculate_path_length(robots[robot_id]) + (DIS_MP_dict[(robots[robot_id].target_workbench_ids[1], workbench_id)] if robots[robot_id].target_workbench_ids[1] != workbench_id else 0)
        # dis = cal_point_x_y(robots[robot_id].x, robots[robot_id].y, workbenchs[workbench_id].x, workbenchs[workbench_id].y)
        if(dis < min_dis):
            min_dis = dis
            min_id = robot_id
    return min_id


def update_GRA_MAP():
    global GRA_MP, Re_GRA_MP
    GRA_MP = np.zeros_like(GRA_MP)
    for idx, workbench in enumerate(workbenchs):
        if(workbench.work_type == 4):
            nearest_1_id = find_nearest_workbench(idx, workbench_type_num[1])
            nearest_2_id = find_nearest_workbench(idx, workbench_type_num[2])
            if nearest_1_id != -1:
                GRA_MP[idx][nearest_1_id] = 1
            if nearest_2_id != -1:
                GRA_MP[idx][nearest_2_id] = 1
        elif(workbench.work_type == 5):
            nearest_1_id = find_nearest_workbench(idx, workbench_type_num[1])
            nearest_3_id = find_nearest_workbench(idx, workbench_type_num[3])
            if nearest_1_id != -1:
                GRA_MP[idx][nearest_1_id] = 1
            if nearest_3_id != -1:
                GRA_MP[idx][nearest_3_id] = 1
        elif(workbench.work_type == 6):
            nearest_2_id = find_nearest_workbench(idx, workbench_type_num[2])
            nearest_3_id = find_nearest_workbench(idx, workbench_type_num[3])
            if nearest_2_id != -1:
                GRA_MP[idx][nearest_2_id] = 1
            if nearest_3_id != -1:
                GRA_MP[idx][nearest_3_id] = 1
        elif(workbench.work_type == 7):
            nearest_4_id = find_nearest_workbench(idx, workbench_type_num[4])
            nearest_5_id = find_nearest_workbench(idx, workbench_type_num[5])
            nearest_6_id = find_nearest_workbench(idx, workbench_type_num[6])
            if nearest_4_id != -1:
                GRA_MP[idx][nearest_4_id] = 1
            if nearest_5_id != -1:
                GRA_MP[idx][nearest_5_id] = 1
            if nearest_6_id != -1:
                GRA_MP[idx][nearest_6_id] = 1
        elif(workbench.work_type == 8):
            nearest_7_id = find_nearest_workbench(idx, workbench_type_num[7])
            if nearest_7_id != -1:
                GRA_MP[idx][nearest_7_id] = 1
        elif(workbench.work_type == 9):
            for id in get_ava_list([1,2,3,4,5,6,7], workbench_type_num):
                if(DIS_MP_dict[(idx, id)]<40000 and can_thing_put_in(id, idx, 1) == 1):
                    GRA_MP[idx][id] = 1
    Re_GRA_MP = GRA_MP.T
    # log.write(f"GRA_MAP IS \n {GRA_MP}\n")

def updata_LOCK_MAP():
    global LOCK_MAP
    st_workbenchs_list = get_ava_list([1, 2, 3, 4, 5, 6, 7, 8, 9], workbench_type_num)
    ed_workbenchs_list = get_ava_list([1, 2, 3, 4, 5, 6, 7, 8, 9], workbench_type_num)
    for st_workbench_id in st_workbenchs_list:
        for ed_workbench_id in ed_workbenchs_list:
            if(can_thing_put_in(st_workbench_id, ed_workbench_id) == 1):
                LOCK_MAP[st_workbench_id][ed_workbench_id] = 1
            else:
                LOCK_MAP[st_workbench_id][ed_workbench_id] = 0
        
def up_down_policy_sxw(free_robots):
    # log.write(f"free robots is {free_robots}\n")
    global GRA_MP, LOCK_MAP, task_cache
    for robot in free_robots:
        for task in task_cache:
            # log.write(f"free robot is {robot}, robots[robot] is {robots[robot].robot_id}, now_suppose_work_space is {robots[robot].now_suppose_work_space}\n")
            if robots[robot].robot_id == task[0]:
                if LOCK_MAP[task[1]][task[2]] == 1:
                    # log.write(f"task_cache is {task_cache}, remove task is {task}\n")
                    task_cache.remove(task)
                    return task[0], task[1], task[2]
                else:
                    # log.write(f"task_cache is {task_cache}, remove task is {task}\n")
                    task_cache.remove(task)
    father_dict = dict()
    task_list = []
    all_workbenchs = get_ava_list([8, 7, 6, 5, 4, 3, 2, 1], workbench_type_num)
    all_workbenchs_sorted = sorted(all_workbenchs, key=functools.cmp_to_key(workbench_cmp3))
    # log.write(f"all_workbenchs_sorted is {all_workbenchs_sorted}\n")
    for workbench_id in all_workbenchs_sorted:
        if(len(np.where(GRA_MP[workbench_id] == 1)[0]) != 0):
            if len(task_list) == 0:
                task_list.append(workbench_id)
            while len(task_list) > 0:
                top_id = task_list.pop(0)
                # log.write(f"top_id is {top_id}\n")
                target_ids = np.where(GRA_MP[top_id] == 1)[0]
                target_ids = sorted(target_ids, key=functools.cmp_to_key(workbench_cmp3))

                for target_id in target_ids:
                    task_list.append(target_id)
                    father_dict[target_id] = top_id
                # task_list = sorted(task_list, key=functools.cmp_to_key(workbench_cmp2))
                
                if(father_dict.get(top_id) != None and LOCK_MAP[top_id][father_dict[top_id]] == 1):
                    # list(set([robot.robot_id for robot in robots]) - set([task[0] for task in task_cache]))
                    # nearest_robot_id = find_nearest_robot_workbench(list(set([robot.robot_id for robot in robots]) - set([task[0] for task in task_cache])), top_id)
                    nearest_robot_id = find_nearest_robot_workbench([robot.robot_id for robot in robots], top_id)
                    if nearest_robot_id != -1:
                        if nearest_robot_id in free_robots:
                            return nearest_robot_id, top_id, father_dict[top_id]
                        else:
                            # log.write(f"task_cache is {task_cache}, append task is {(nearest_robot_id, top_id, father_dict[top_id])}\n")
                            for task in task_cache:
                                if nearest_robot_id == task[0]:
                                    if DIS_MP_dict[(robots[nearest_robot_id].target_workbench_ids[1], task[1])] > DIS_MP_dict[(robots[nearest_robot_id].target_workbench_ids[1], top_id)]:
                                        task_cache.remove(task)
                                        task_cache.append((nearest_robot_id, top_id, father_dict[top_id]))
                                    break
                else:
                    min_dis = 40000
                    min_id = -1
                    for id in get_ava_list([9], workbench_type_num):
                        if(LOCK_MAP[top_id][id] == 1 and DIS_MP_dict[(top_id, id)] < min_dis):
            
                            min_dis = DIS_MP_dict[(top_id, id)]
                            min_id = id
                    if(min_id != -1):
                        # nearest_robot_id = find_nearest_robot_workbench(list(set([robot.robot_id for robot in robots]) - set([task[0] for task in task_cache])), top_id)
                        nearest_robot_id = find_nearest_robot_workbench([robot.robot_id for robot in robots], top_id)
                        if nearest_robot_id != -1:
                            if nearest_robot_id in free_robots:
                                return nearest_robot_id, top_id, min_id
                            else:
                                # log.write(f"task_cache is {task_cache}, append task is {(nearest_robot_id, top_id, min_id)}\n")
                                for task in task_cache:
                                    if nearest_robot_id == task[0]:
                                        if DIS_MP_dict[(robots[nearest_robot_id].target_workbench_ids[1], task[1])] > DIS_MP_dict[(robots[nearest_robot_id].target_workbench_ids[1], top_id)]:
                                            task_cache.remove(task)
                                            task_cache.append((nearest_robot_id, top_id, father_dict[top_id]))
                                        break
    return -1, -1, -1        

def bfs_half_init():
    global env_mp, workbench_taking_mp, index_taking_mp
    global workbench_nothing_mp, index_nothing_mp, useful_workbench_cnt
    for id in range(workbench_ids):
        if refuse_workbench_dist[id] == 1:
            continue
        workbench_taking_mp[id], index_taking_mp[id] = bfs_half(env_mp, (workbenchs[id].half_x, workbenchs[id].half_y), 1, map_limit)
        if useful_workbench_cnt <= 13:
            workbench_nothing_mp[id], index_nothing_mp[id] = bfs_half(env_mp, (workbenchs[id].half_x, workbenchs[id].half_y), 0, map_limit)
        
        # log.write(f"{workbench_taking_mp[id][(workbenchs[id].half_x, workbenchs[id].half_y)]}\n")

def workbench_after_half_init():

    if workbench_mode == 1:
        path_better_nothing_mp[2][0] = [(45.25, 48.75), (25.25, 49.25)]
        path_better_nothing_mp[3][1] = [(6.75, 48.25), (4.25, 48.75)]
        path_better_nothing_mp[5][1] = [(1.75, 38.25), (4.25, 48.75)]
        path_better_nothing_mp[6][4] = [(1.25, 1.25), (1.75, 43.25)]
        path_better_nothing_mp[6][6] = [(1.25, 1.25)]
        path_better_nothing_mp[7][0] = [(48.75, 1.25), (42.75, 8.0), (25.25, 25.5), (25.25, 49.25)]
        path_better_nothing_mp[7][4] = [(48.75, 1.25), (25.25, 19.75), (1.75, 43.25)]
        path_better_nothing_mp[7][7] = [(48.75, 1.25)]
        
        path_better_taking_mp[0][7] = [(25.25, 49.25), (25.25, 27.5), (42.75, 8.0), (48.75, 1.25)]

        path_better_taking_mp[1][6] = [(4.25, 48.75), (1.25, 1.25)]

        path_better_taking_mp[4][6] = [(1.75, 43.25), (1.25, 1.25)]
        path_better_taking_mp[4][7] = [(1.75, 43.25), (34.0, 12.75), (41.25, 7.0), (48.75, 1.25)]

        path_better_taking_mp[6][2] = [(1.25, 1.25), (1.25, 4.0), (34.75, 38.0), (45.25, 48.75)]
        path_better_taking_mp[6][3] = [(1.25, 1.25), (6.75, 48.25)]

        path_better_taking_mp[6][5] = [(1.25, 1.25), (1.75, 38.25)]

        path_better_taking_mp[7][3] = [(48.75, 1.25), (6.75, 48.25)]

        path_better_taking_mp[7][5] = [(48.75, 1.25), (45.0, 1.25), (25.25, 14.75), (1.75, 38.25)]
    for id0 in range(workbench_ids):
        # if refuse_workbench_dist[id0] == 1:
        #     continue
        for id1 in range(id0, workbench_ids):
            if refuse_workbench_dist[id1] == 1 or refuse_workbench_dist[id0] == 1:
                DIS_MP_dict[(id0, id1)] = DIS_MP_dict[(id1, id0)] = 40000
                continue
            # log.write(f"{id0, id1}\n")
            # log.write(f"{index_taking_mp[id0]}")
            if index_taking_mp[id0].get((workbenchs[id1].half_x, workbenchs[id1].half_y)) is None:
                DIS_MP_dict[(id0, id1)] = DIS_MP_dict[(id1, id0)] = 40000
            else:
                DIS_MP_dict[(id0, id1)] = DIS_MP_dict[(id1, id0)] = index_taking_mp[id0][(workbenchs[id1].half_x, workbenchs[id1].half_y)] * 0.5
            if useful_workbench_cnt <= 13:
                if index_nothing_mp[id0].get((workbenchs[id1].half_x, workbenchs[id1].half_y)) is None:
                    DIS_MP_nothing_dict[(id0, id1)] = DIS_MP_nothing_dict[(id1, id0)] = 40000
                else:
                    DIS_MP_nothing_dict[(id0, id1)] = DIS_MP_nothing_dict[(id1, id0)] = index_nothing_mp[id0][(workbenchs[id1].half_x, workbenchs[id1].half_y)] * 0.5
    for workbench_a in range(workbench_ids):
        if refuse_workbench_dist[workbench_a] == 1:
            continue
        target_workbench_type = choose_target_workbench_list(generate_product, workbenchs[workbench_a].work_type)
        cnt = -1
        for type in target_workbench_type:
            workbench_minest_sell[workbench_a].append(-1)
            cnt += 1
            for workbench_b in workbench_type_num[type]:
                if refuse_workbench_dist[workbench_b] == 1:
                    continue
                if workbench_minest_sell[workbench_a][cnt] == -1 or DIS_MP_dict[(workbench_a, workbench_b)] < DIS_MP_dict[(workbench_a, workbench_minest_sell[workbench_a][cnt])]:
                    workbench_minest_sell[workbench_a][cnt] = workbench_b
            

def robot_half_bfs_init():
    global env_mp, robot_taking_mp, robot_index_taking_mp, workbenchs, useful_workbench_cnt
    for id in range(4):
        if id != 3 and workbench_mode == 1:
            continue
        nx, ny = robots[id].half_x, robots[id].half_y
        robot_taking_mp[id], robot_index_taking_mp[id] = bfs_half(env_mp, (nx, ny), 0, map_limit)

    for id in range(workbench_ids):
        workbench = workbenchs[id]
        if workbench_mode != 1:
            if robot_taking_mp[0].get((workbench.half_x, workbench.half_y)) is None and robot_taking_mp[1].get((workbench.half_x, workbench.half_y)) is None and robot_taking_mp[2].get((workbench.half_x, workbench.half_y)) is None and robot_taking_mp[3].get((workbench.half_x, workbench.half_y)) is None:
                refuse_workbench_dist[id] = 1
            else:
                useful_workbench_cnt += 1
                refuse_workbench_dist[id] = 0
            for i in range(4):
                if robot_taking_mp[i].get((workbench.half_x, workbench.half_y)) is None:
                    robot_index_taking_mp[i][(workbench.half_x, workbench.half_y)] = 40000
        else:
            if robot_taking_mp[3].get((workbench.half_x, workbench.half_y)) is None:
                refuse_workbench_dist[id] = 1
            else:
                useful_workbench_cnt += 1
                refuse_workbench_dist[id] = 0
            for i in range(3, 4):
                if robot_taking_mp[i].get((workbench.half_x, workbench.half_y)) is None:
                    robot_index_taking_mp[i][(workbench.half_x, workbench.half_y)] = 40000
        
        


def find_free_workbench(st_workbench_type, ed_workbench):
    minest_dis = 40000
    minest_no = -1
    for id in workbench_type_num[st_workbench_type]:
        if workbenchs[id].is_waiting == 1 :
            continue
        temp_dis = DIS_MP_dict[(id, ed_workbench)]
        if temp_dis < minest_dis:
            minest_dis = temp_dis
            minest_no = id
    return minest_no

def workbench_cmp(x, y):
    if cfg.THING_VALUE[workbenchs[x[0]].work_type] > cfg.THING_VALUE[workbenchs[y[0]].work_type]:
        return -1
    if cfg.THING_VALUE[workbenchs[x[0]].work_type] < cfg.THING_VALUE[workbenchs[y[0]].work_type]:
        return 1
    return 0

def update_task_list():
    global task_list, generate_list, finished_list, waiting_list
    task_len = len(task_list)
    new_task_list = []
    flags = [0 for i in range(task_len)]
    for id in range(task_len):
        new_task_list.append(task_list[id])

    for id in range(len(new_task_list)):
        work_type = workbenchs[new_task_list[id][0]].work_type
        if work_type == 7 and ((1 << 4) * workbenchs[new_task_list[id][0]].up_down_flag[4] + (1 << 5) * workbenchs[new_task_list[id][0]].up_down_flag[5] + (1 << 6) * workbenchs[new_task_list[id][0]].up_down_flag[6]) | workbenchs[new_task_list[id][0]].origin_thing == 112:
            flags[id] = 1
            waiting_list.append(new_task_list[id])
            workbenchs[new_task_list[id][0]].is_waiting = 2
        if work_type == 6 and ((1 << 2) * workbenchs[new_task_list[id][0]].up_down_flag[2] + (1 << 3) * workbenchs[new_task_list[id][0]].up_down_flag[3]) | workbenchs[new_task_list[id][0]].origin_thing == 12:
            flags[id] = 1
            waiting_list.append(new_task_list[id])
            workbenchs[new_task_list[id][0]].is_waiting = 2
        if work_type == 5 and ((1 << 1) * workbenchs[new_task_list[id][0]].up_down_flag[1] + (1 << 3) * workbenchs[new_task_list[id][0]].up_down_flag[3]) | workbenchs[new_task_list[id][0]].origin_thing == 10:
            flags[id] = 1
            waiting_list.append(new_task_list[id])
            workbenchs[new_task_list[id][0]].is_waiting = 2
        if work_type == 4 and ((1 << 1) * workbenchs[new_task_list[id][0]].up_down_flag[1] + (1 << 2) * workbenchs[new_task_list[id][0]].up_down_flag[2]) | workbenchs[new_task_list[id][0]].origin_thing == 6:
            flags[id] = 1
            waiting_list.append(new_task_list[id])
            workbenchs[new_task_list[id][0]].is_waiting = 2
    new_task_list = []
    for i in range(task_len):
        if flags[i] == 0:
            new_task_list.append(task_list[i])
    task_list = new_task_list

    waiting_len = len(waiting_list)
    new_waiting_list = []
    flags = [0 for i in range(waiting_len)]
    for id in range(waiting_len):
        new_waiting_list.append(waiting_list[id])

    for id in range(len(new_waiting_list)):
        work_type = workbenchs[new_waiting_list[id][0]].work_type
        if workbenchs[new_waiting_list[id][0]].origin_thing == 0 and workbenchs[new_waiting_list[id][0]].remain_time > 0:
            flags[id] = 1
            generate_list.append(new_waiting_list[id])
            # workbenchs[new_waiting_list[id][0]].is_waiting = 0

    new_waiting_list = []
    for i in range(waiting_len):
        if flags[i] == 0:
            new_waiting_list.append(waiting_list[i])
    waiting_list = new_waiting_list

    generate_len = len(generate_list)
    new_generate_list = []
    flags = [0 for i in range(generate_len)]
    for id in range(generate_len):
        new_generate_list.append(generate_list[id])

    for id in range(len(new_generate_list)):
        if workbenchs[new_generate_list[id][0]].output == 1 or 0 < workbenchs[new_generate_list[id][0]].remain_time < 100:
            flags[id] = 1
            workbenchs[new_generate_list[id][0]].up_down_flag = [0 for i in range(10)]
        
            workbenchs[new_generate_list[id][0]].is_waiting = 0
            finished_list.append(new_generate_list[id])
    new_generate_list = []
    for i in range(generate_len):
        if flags[i] == 0:
            new_generate_list.append(generate_list[i])
    generate_list = new_generate_list

    finished_list = sorted(finished_list, key=functools.cmp_to_key(workbench_cmp))
    # log.write(f"-\n")
    # log.write(f"{workbenchs[workbench_type_num[7][0]].up_down_flag}")
    # log.write(f"{task_list} \n")
    # log.write(f"{waiting_list}\n")
    # log.write(f"{generate_list} \n")
    # log.write(f"{finished_list} \n")
    # log.write(f"-\n")

def up_down_policy(free_robots):
    global task_list, generate_list, finished_list
    robot_id, target0_id, target1_id = -1, -1, -1
    while robot_id == -1:
        task_pos = len(task_list) - 1
        finished_pos = len(finished_list) - 1
        if finished_pos == -1:
            if task_pos == -1:
                # 找最高级任务
                flag = 0
                for task_type in cfg.TASK_TYPE:
                    for workbench in workbench_type_num[task_type]:
                        if workbenchs[workbench].is_waiting == 0:
                            if workbenchs[workbench].work_type == 7:
                                targeted_work_type = [8, 9]
                            # else:
                            #     continue
                            elif workbenchs[workbench].work_type in [4, 5, 6]:
                                targeted_work_type = [7, 9]
                            elif workbenchs[workbench].work_type == 1:
                                targeted_work_type = [4, 5, 9]
                            elif workbenchs[workbench].work_type == 2:
                                targeted_work_type = [4, 6, 9]
                            elif workbenchs[workbench].work_type == 3:
                                targeted_work_type = [5, 6, 9]
                            
                            best_dis, best_id = 40000, -1
                            for i in targeted_work_type:
                                for j in workbench_type_num[i]:
                                    temp_dis = DIS_MP_dict[(workbench, j)]
                                    if temp_dis < best_dis:
                                        best_dis = temp_dis
                                        best_id = j
                            task_list.append((workbench, best_id))
                            workbenchs[workbench].is_waiting = 1
                            flag = 1
                            break
                    if flag == 1:
                        break
            elif workbenchs[task_list[task_pos][0]].work_type == 7:
                # 当任务列表的首部未生产完毕任务为7时
                if workbenchs[task_list[task_pos][0]].origin_thing != 112:
                    # 还没开始生产 找对应 6 5 4
                    workbench_append_id = -1
                    if(1 << 6) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[6] == 0:
                        workbench_append_id = find_free_workbench(6, task_list[task_pos][0])
                    if (1 << 5) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[5] == 0:
                        workbench_append_id = find_free_workbench(5, task_list[task_pos][0])
                    if (1 << 4) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[4] == 0:
                        workbench_append_id = find_free_workbench(4, task_list[task_pos][0])
                    task_list.append((workbench_append_id, task_list[task_pos][0]))
                    workbenchs[task_list[task_pos][0]].up_down_flag[workbenchs[workbench_append_id].work_type] = 1
                    # log.write(f"~{task_list[task_pos][0]} {workbenchs[workbench_append_id].work_type}\n")

            elif workbenchs[task_list[task_pos][0]].work_type == 6:
                # 为6
                if workbenchs[task_list[task_pos][0]].origin_thing != 12:
                    # 还没开始生产 找对应 3 2
                    workbench_append_id = -1
                    if (1 << 3) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[3] == 0:
                        workbench_append_id = find_free_workbench(3, task_list[task_pos][0])
                    if (1 << 2) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[2] == 0:
                        workbench_append_id = find_free_workbench(2, task_list[task_pos][0])
                    task_list.append((workbench_append_id, task_list[task_pos][0]))
                    workbenchs[task_list[task_pos][0]].up_down_flag[workbenchs[workbench_append_id].work_type] = 1
                    
                    # log.write(f"{task_list} {task_pos}\n")
            elif workbenchs[task_list[task_pos][0]].work_type == 5:
                    # 还没开始生产 找对应 3 1
                if workbenchs[task_list[task_pos][0]].origin_thing != 10:
                    workbench_append_id = -1
                    if (1 << 3) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[3] == 0:
                        workbench_append_id = find_free_workbench(3, task_list[task_pos][0])
                    if (1 << 1) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[1] == 0:
                        workbench_append_id = find_free_workbench(1, task_list[task_pos][0])
                    task_list.append((workbench_append_id, task_list[task_pos][0]))
                    workbenchs[task_list[task_pos][0]].up_down_flag[workbenchs[workbench_append_id].work_type] = 1
                    
                    task_pos += 1
            elif workbenchs[task_list[task_pos][0]].work_type == 4:
                # 还没开始生产 找对应 2 1
                if workbenchs[task_list[task_pos][0]].origin_thing != 6:
                    workbench_append_id = -1
                    if (1 << 2) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[2] == 0:
                        workbench_append_id = find_free_workbench(2, task_list[task_pos][0])
                    if (1 << 1) & workbenchs[task_list[task_pos][0]].origin_thing == 0 and workbench_append_id == -1 and workbenchs[task_list[task_pos][0]].up_down_flag[1] == 0:
                        workbench_append_id = find_free_workbench(1, task_list[task_pos][0])
                    task_list.append((workbench_append_id, task_list[task_pos][0]))
                    workbenchs[task_list[task_pos][0]].up_down_flag[workbenchs[workbench_append_id].work_type] = 1

            else:
                # task_list为3 2 1
                finished_tuple = task_list.pop(task_pos)
                finished_list.append(finished_tuple)
                finished_list = sorted(finished_list, key=functools.cmp_to_key(workbench_cmp))
            update_task_list()
        else:
            best_dis = 40000
            best_id = -1
            finished_id = finished_list[0]
            finished_list.pop(0)
            for id in free_robots:
                temp_dis = robot_index_taking_mp[id][(workbenchs[finished_id[0]].half_x, workbenchs[finished_id[0]].half_y)]
                # cal_point_x_y(robots[id].x, robots[id].y, workbenchs[finished_id[0]].x, workbenchs[finished_id[0]].y)
                if best_dis > temp_dis:
                    best_dis = temp_dis
                    best_id = id
            workbenchs[finished_id[0]].is_waiting = 0
            robot_id, target0_id, target1_id = best_id, finished_id[0], finished_id[1]
            # log.write(f"{robot_id}, {target0_id}, {target1_id}\n")

    return robot_id, target0_id, target1_id    


# Main
if __name__ == '__main__':
    # input env_map
    d1 = time.time()
    env_mp, GRA_MP, Re_GRA_MP, LOCK_MAP = read_map(),\
                                                  [[0 for j in range(50)] for i in range(50)],\
                                                  [[0 for j in range(50)] for i in range(50)],\
                                                  [[1 for j in range(50)] for i in range(50)]
    map_init()

    d2 = time.time()
    robot_half_bfs_init()
    # robot_bfs_init()
    # robot_astar_init()
    d3 = time.time()
    # bfs_init()
    bfs_half_init()
    # astar_init()
    d4 = time.time()
    finish()
    workbench_after_half_init()
    bfs_half(env_mp, (workbenchs[2].half_x, workbenchs[2].half_y), 1, map_limit, debug_mode=1)

    while True:
        line = sys.stdin.readline()
        if not line:
            break
        # input every frame
        parts = line.split(' ')
        frame_id, money_frame = int(parts[0]), int(parts[1])
        # update
        workbench_frame_num, workbench_id = int(input()), -1
        for workbench in range(workbench_frame_num):
            workbench_type, workbench_x, workbench_y, workbench_remain, workbench_origin, workbench_output = input().split()
            # create a new craft table
            workbench_id += 1
            # update it
            workbenchs[workbench_id].get_from_frame(workbench_type, workbench_x, workbench_y, workbench_remain, workbench_origin, workbench_output)
        
        for robot in range(cfg.ROBOT_NUM):
            robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y = input().split()
            # update the robot state
            robots[robot].get_from_frame(robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y)
            robots[robot].anti_x, robots[robot].anti_y = int(anti_cal_y(robots[robot].y) * 2), int(anti_cal_x(robots[robot].x) * 2)
            robots[robot].half_x, robots[robot].half_y = int(anti_cal_y(robots[robot].y)), int(anti_cal_x(robots[robot].x))
        for idx, robot in enumerate(robots):
            wall_list[idx].clear()
            for wall in walls:
                if cal_point_x_y(robot.x, robot.y, wall.x, wall.y) < 1.5:
                   wall_list[idx].append(wall) 
        # 分配任务
        free_robots = find_free_robot(robots, workbench_mode)
        # free_jobs = find_free_job(workbenchs)
        
        # if workbench_mode == 1:
        update_task_list()

        free_robot_len = len(free_robots)
        for i in range(free_robot_len):
            # if workbench_mode == 1
            # log.write(f"{employ_robot, target0, target1}!!!!!!!\n")
            # employ_robot, target0, target1 = get_price_by_targets(free_robots, 2, frame_id)
            # if employ_robot == -1:
            #     employ_robot, target0, target1 = get_price_by_targets(free_robots, 1, frame_id)
            # if workbench_mode == 2 and workbench_mode != 1:
            #     employ_robot, target0, target1 = up_down_policy(free_robots)
            # if len(workbench_mode):
            #     updata_LOCK_MAP()
            #     update_GRA_MAP()
            #     employ_robot, target0, target1 = up_down_policy_sxw(free_robots)
            # else:
            employ_robot, target0, target1 = get_price_by_targets_half(free_robots, 2, frame_id)
            if employ_robot == -1:
                employ_robot, target0, target1 = get_price_by_targets_half(free_robots, 1, frame_id)

            # log.write(f"{employ_robot, target0, target1}\n")
            if employ_robot != -1:
                robots[employ_robot].target_workbench_ids[0] = target0

                ### 更新到target0的路径move_list_target0
                work_space = robots[employ_robot].now_suppose_work_space
                if useful_workbench_cnt > 13:
                    if work_space == -1:
                        robots[employ_robot].move_list_target0 = ask_path_sxw((workbenchs[target0].half_x, workbenchs[target0].half_y), robot_taking_mp[employ_robot], 0, walls)

                    elif path_better_taking_mp[work_space][target0] is None:
                        path_better_taking_mp[work_space][target0] = ask_path_sxw((workbenchs[target0].half_x, workbenchs[target0].half_y), workbench_taking_mp[work_space], 1, walls)
                        robots[employ_robot].move_list_target0 = path_better_taking_mp[work_space][target0]
                    else:
                        robots[employ_robot].move_list_target0 = path_better_taking_mp[work_space][target0]
                else:
                    if work_space == -1:
                        if workbench_mode != 1:
                            robots[employ_robot].move_list_target0 = ask_path_sxw((workbenchs[target0].half_x, workbenchs[target0].half_y), robot_taking_mp[employ_robot], 0, walls)
                        elif target0 == 4:
                            robots[employ_robot].move_list_target0 = [(robots[employ_robot].x, robots[employ_robot].y), (workbenchs[target0].x, workbenchs[target0].y)]
                    elif path_better_nothing_mp[work_space][target0] is None:
                        path_better_nothing_mp[work_space][target0] = ask_path_sxw((workbenchs[target0].half_x, workbenchs[target0].half_y), workbench_nothing_mp[work_space], 0, walls)
                        robots[employ_robot].move_list_target0 = path_better_nothing_mp[work_space][target0]
                    else:
                        robots[employ_robot].move_list_target0 = path_better_nothing_mp[work_space][target0]
                ###

                ### 更新target0到target1的路径move_list_target1
                robots[employ_robot].target_workbench_ids[1] = target1
                if path_better_taking_mp[target0][target1] is None:
                    # path_better_taking_mp[target0][target1] = path_better(new_env_mp, astar_workbench_nothing_mp[target0][target1], 1, mask_env_mp)
                    # path_better_taking_mp[target0][target1] = ask_path_half((workbenchs[target1].half_x, workbenchs[target1].half_y), workbench_taking_mp[target0], env_mp, 1)
                    path_better_taking_mp[target0][target1] = ask_path_sxw((workbenchs[target1].half_x, workbenchs[target1].half_y), workbench_taking_mp[target0], 1, walls)
                    
                    # path_better_taking_mp[target0][target1] = ask_path((workbenchs[target1].anti_x, workbenchs[target1].anti_y), workbench_taking_mp[target0], new_env_mp, mask_env_mp)
                    # path_better_taking_mp[target0][target1] = ask_path_sxw((workbenchs[target1].anti_x, workbenchs[target1].anti_y), workbench_taking_mp[target0], new_env_mp, mask_env_mp, 1)
                    robots[employ_robot].move_list_target1 = path_better_taking_mp[target0][target1]
                else:
                    robots[employ_robot].move_list_target1 = path_better_taking_mp[target0][target1]
                ###
                if workbench_mode not in [1, 2]:
                    update_path_lock(robots[employ_robot].move_list_target0 + robots[employ_robot].move_list_target1, employ_robot)
                workbenchs[target0].is_targeted_flag[0] = 1
                workbenchs[target1].is_targeted_flag[workbenchs[target0].work_type] = 1
                # if workbenchs[robots[employ_robot].target_workbench_ids[0]].work_type in [1, 2, 3]:
                #     workbenchs[robots[employ_robot].target_workbench_ids[0]].is_targeted_flag[0] = 0
                robots[employ_robot].move_list_target0 = deque(robots[employ_robot].move_list_target0)
                robots[employ_robot].move_list_target0.append((-1, -1))
                robots[employ_robot].move_list_target1 = deque(robots[employ_robot].move_list_target1)
                robots[employ_robot].move_list_target1.append((-1, -1))
                # log.write(f"{robots[employ_robot].move_list_target0}\n")
                # log.write(f"{robots[employ_robot].move_list_target1}\n")
                free_robots.remove(employ_robot)
            update_task_list()

        line = sys.stdin.readline()
        # log.write(f'{robots[0].target_workbench_ids[0], robots[0].target_workbench_ids[1]}\n{robots[0].move_list_target0}\n')
        # exit()
        # do some operation
        sys.stdout.write('%d\n' % frame_id)
        for robot_id in range(cfg.ROBOT_NUM):
            if robot_id != 3 and workbench_mode == 1:
                continue
            # if robot_id != 1:
            #     continue
            # log.write(f"{robot_id, robots[robot_id].x, robots[robot_id].y, robots[robot_id].state}\n")
            # log.write(f'{robots[robot_id].move_list_target0}\n')
            # log.write(f'{robots[robot_id].move_list_target1}\n')
            # log.write(f'{robots[robot_id].move_history}\n')
            # log.write(f"~~~~~~~~~~~~~~~~~~~~~~~~\n")
            rotate, forward = None, None
            if robots[robot_id].target_workbench_ids[0] == -1:
                continue
            if robots[robot_id].work_frame <= frame_id:
                if robots[robot_id].state == 0:
                    # calc the speed and go
                    # distance to target
                    temp_target = robots[robot_id].move_list_target0[0]
                    end_flag = 1 if robots[robot_id].move_list_target0[1] == (-1, -1) else 0
                    distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # direction to target
                    direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # remain_path_len = len(robots[robot_id].move_list_target0)
                    
                    # reach the target workbench
                    if end_flag == 1 and robots[robot_id].work_space == robots[robot_id].target_workbench_ids[0]:
                            robots[robot_id].s_pid.clear()
                            robots[robot_id].w_pid.clear()
                            robots[robot_id].move_list_target0.rotate(-1)
                            robots[robot_id].now_suppose_work_space = robots[robot_id].target_workbench_ids[0]
                            # sys.stdout.write('forward %d %f\n' % (robot_id, 0))
                            cfg.pid_list[robot_id] = [0, 0]
                            robots[robot_id].state = 1
                            battle_time[robot_id] = 0
                            robots[robot_id].move_history.clear()
                            robots[robot_id].line_flag = 0
                            # robots[robot_id].move_list_target0.rotate(-1)
                    elif end_flag == 0 and distance <= cfg.DIS_TOLERATE:
                            robots[robot_id].move_list_target0.rotate(-1)

                            robots[robot_id].move_history.clear()
                            robots[robot_id].line_flag = 0
                    else:
                        robots[robot_id].move_history.appendleft((robots[robot_id].half_x, robots[robot_id].half_y))
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        if (abs(direction - robots[robot_id].toward) <= cfg.PI / 4 and workbench_mode != 1) or(workbench_mode == 1 and abs(direction - robots[robot_id].toward) <= cfg.PI / 4):
                            robots[robot_id].line_flag = 1
                            cfg.pid_list[robot_id] = [rotate, forward]
                        else:
                            cfg.pid_list[robot_id] = [rotate, 0]
                        # cfg.pid_list[robot_id] = [rotate, forward]
                        if robots[robot_id].move_list_target0[0] != (-1, -1):
                            point = robots[robot_id].move_list_target0[0]
                            if mask_path_mp[robot_id][anti_cal_y(point[1])][anti_cal_x(point[0])] == 1:
                                cfg.pid_list[robot_id] = [0, 0]
                        elif robots[robot_id].move_list_target1[0] != (-1, -1):
                            point = robots[robot_id].move_list_target1[0]
                            if mask_path_mp[robot_id][anti_cal_y(point[1])][anti_cal_x(point[0])] == 1:
                                cfg.pid_list[robot_id] = [0, 0]
                    speedx = robots[robot_id].line_speed_x
                    speedy = robots[robot_id].line_speed_y
                    if cfg.pid_list[robot_id][1] != 0 and sqrt(speedx * speedx + speedy * speedy) < 0.5:
                        battle_time[robot_id] += 1

                elif robots[robot_id].state == 1:
                    # buy
                    if workbenchs[robots[robot_id].target_workbench_ids[0]].output == 1 and robots[robot_id].work_space == robots[robot_id].target_workbench_ids[0]:
                        if DIS_MP_dict[(robots[robot_id].target_workbench_ids[0], robots[robot_id].target_workbench_ids[1])] / 5.5 > (150000 - frame_id) / 50 - 1:
                            # go to 8,9
                            continue
                        sys.stdout.write('buy %d\n' % robot_id)
                        workbenchs[robots[robot_id].target_workbench_ids[0]].is_targeted_flag[0] = 0
                        robots[robot_id].state = 2
                    else:
                        robots[robot_id].move_list_target0.rotate(1)
                        robots[robot_id].state = 0
                elif robots[robot_id].state == 2:
                    # calc the speed and go
                    # distance to target
                    temp_target = robots[robot_id].move_list_target1[0]
                    end_flag = 1 if robots[robot_id].move_list_target1[1] == (-1, -1) else 0
                    distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # direction to target
                    direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # remain_path_len = len(robots[robot_id].move_list_target1)
                    
                    # reach the target workbench
                    if end_flag == 1 and robots[robot_id].work_space == robots[robot_id].target_workbench_ids[1]:
                            robots[robot_id].s_pid.clear()
                            robots[robot_id].w_pid.clear()
                            robots[robot_id].now_suppose_work_space = robots[robot_id].target_workbench_ids[1]
                            robots[robot_id].move_list_target1.rotate(-1)
                            cfg.pid_list[robot_id] = [0, 0]
                            # sys.stdout.write('forward %d %f\n' % (robot_id, 0))
                            robots[robot_id].state = 3
                            battle_time[robot_id] = 0
                            robots[robot_id].move_history.clear()
                            robots[robot_id].line_flag = 0
                    elif end_flag == 0 and distance <= cfg.DIS_TOLERATE:
                            robots[robot_id].move_list_target1.rotate(-1)
                        # cfg.pid_list[robot_id] = [rotate, forward]
                            robots[robot_id].move_history.clear()
                            robots[robot_id].line_flag = 0
                    else:
                        robots[robot_id].move_history.appendleft((robots[robot_id].half_x, robots[robot_id].half_y))
                        # robots[robot_id].move_history.appendleft((robots[robot_id].anti_x, robots[robot_id].anti_y))
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        # log.write(f'direction:{robot_id} {direction} {robots[robot_id].toward}\n')
                        if (abs(direction - robots[robot_id].toward) <= cfg.PI / 4 and workbench_mode != 1) or (workbench_mode == 1 and abs(direction - robots[robot_id].toward) <= cfg.PI / 4):
                            robots[robot_id].line_flag = 1
                            cfg.pid_list[robot_id] = [rotate, forward]
                            # log.write(f'pid_list:{robot_id} {cfg.pid_list[robot_id][0]} {cfg.pid_list[robot_id][1]}\n')
                        else:
                            cfg.pid_list[robot_id] = [rotate, 0]
                        if robots[robot_id].move_list_target0[0] != (-1, -1):
                            point = robots[robot_id].move_list_target0[0]
                            if mask_path_mp[robot_id][anti_cal_y(point[1])][anti_cal_x(point[0])] == 1:
                                cfg.pid_list[robot_id] = [0, 0]
                        elif robots[robot_id].move_list_target1[0] != (-1, -1):
                            point = robots[robot_id].move_list_target1[0]
                            if mask_path_mp[robot_id][anti_cal_y(point[1])][anti_cal_x(point[0])] == 1:
                                cfg.pid_list[robot_id] = [0, 0]
                        # cfg.pid_list[robot_id] = [rotate, forward]
                    speedx = robots[robot_id].line_speed_x
                    speedy = robots[robot_id].line_speed_y
                    if cfg.pid_list[robot_id][1] != 0 and sqrt(speedx * speedx + speedy * speedy) < 0.5:
                        battle_time[robot_id] += 1
                    #     if battle_time[robot_id] >= cfg.BATTLE_TIME[robots[robot_id].take_thing]:
                    #         ## 回退一步
                    #         # robots[robot_id].move_history = deque(path_better(env_mp, robots[robot_id].move_history, 1, mask_env_mp))
                    #         # robots[robot_id].move_history.append((-1, -1))
                    #         # if robots[robot_id].move_list_target0[-1] != (-1, -1):
                    #         #     robots[robot_id].move_list_target0.rotate(1)
                    #         # robots[robot_id].state = 7
                    #         ###

                    #         ## 回退一大步
                    #         robots[robot_id].state = 5
                            ###
                    # log.write(f'robotid:{robot_id} {cfg.pid_list[robot_id][0]} {cfg.pid_list[robot_id][1]}\n')
                elif robots[robot_id].state == 3:
                    # sell and turn 0
                    take_thing = robots[robot_id].take_thing
                    target = robots[robot_id].target_workbench_ids[1]
                    if robots[robot_id].work_space == target and not ((1 << take_thing) & workbenchs[target].origin_thing):
                        sys.stdout.write('sell %d\n' % robot_id)
                        # 将相应原料的卖操作解锁
                        # 1111110
                        robots[robot_id].value = 0
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
                        robots[robot_id].move_list_target1.rotate(1)
                        robots[robot_id].state = 2
                elif robots[robot_id].state == 4:
                    temp_target = robots[robot_id].move_list_target0[-1]
                    end_flag = 1 if robots[robot_id].move_list_target0[-2] == (-1, -1) else 0
                    distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # direction to target
                    direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # remain_path_len = len(robots[robot_id].move_list_target1)
                    
                    # reach the target workbench
                    if (back_distannce[robot_id] >= cfg.BACK_DISTANCE or end_flag == 1) and distance <= cfg.DIS_TOLERATE:
                            robots[robot_id].s_pid.clear()
                            robots[robot_id].w_pid.clear()
                            robots[robot_id].move_list_target0.rotate(1)
                            cfg.pid_list[robot_id] = [0, 0]
                            battle_time[robot_id] = 0
                            back_distannce[robot_id] = 0
                            robots[robot_id].state = 0
                    elif end_flag == 0 and distance <= cfg.DIS_TOLERATE:
                            back_distannce[robot_id] += cal_point_x_y(robots[robot_id].move_list_target0[0][0], robots[robot_id].move_list_target0[0][1], robots[robot_id].move_list_target0[-1][0], robots[robot_id].move_list_target0[-1][1])
                            robots[robot_id].move_list_target0.rotate(1)
                    else:
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        if (abs(direction - robots[robot_id].toward) <= cfg.PI / 4 and workbench_mode != 1) or(workbench_mode == 1 and abs(direction - robots[robot_id].toward) <= cfg.PI / 4):
                            cfg.pid_list[robot_id] = [rotate, forward]
                        else:
                            cfg.pid_list[robot_id] = [rotate, 0]
                        # cfg.pid_list[robot_id] = [rotate, forward]
                elif robots[robot_id].state == 5:
                    temp_target = robots[robot_id].move_list_target1[-1]
                    end_flag = 1 if robots[robot_id].move_list_target1[-2] == (-1, -1) else 0
                    distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # direction to target
                    direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # remain_path_len = len(robots[robot_id].move_list_target1)
                    
                    # reach the target workbench
                    if (back_distannce[robot_id] >= cfg.BACK_DISTANCE or end_flag == 1) and distance <= cfg.DIS_TOLERATE:
                            robots[robot_id].s_pid.clear()
                            robots[robot_id].w_pid.clear()
                            robots[robot_id].move_list_target1.rotate(1)
                            cfg.pid_list[robot_id] = [0, 0]
                            battle_time[robot_id] = 0
                            back_distannce[robot_id] = 0
                            robots[robot_id].state = 2
                    elif end_flag == 0 and distance <= cfg.DIS_TOLERATE:
                            back_distannce[robot_id] += cal_point_x_y(robots[robot_id].move_list_target1[0][0], robots[robot_id].move_list_target1[0][1], robots[robot_id].move_list_target1[-1][0], robots[robot_id].move_list_target1[-1][1])
                            robots[robot_id].move_list_target1.rotate(1)
                    else:
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        if (abs(direction - robots[robot_id].toward) <= cfg.PI / 4 and workbench_mode !=1) or(workbench_mode == 1 and abs(direction - robots[robot_id].toward) <= cfg.PI / 4):
                            cfg.pid_list[robot_id] = [rotate, forward]
                        else:
                            cfg.pid_list[robot_id] = [rotate, 0]
                        # cfg.pid_list[robot_id] = [rotate, forward]
                elif robots[robot_id].state == 6:
                    temp_target = robots[robot_id].move_history[0]
                    end_flag = 1 if robots[robot_id].move_history[1] == (-1, -1) else 0
                    distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # direction to target
                    direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # remain_path_len = len(robots[robot_id].move_list_target1)

                    # reach the target workbench
                    if distance <= cfg.DIS_TOLERATE:
                        if end_flag == 1:
                            robots[robot_id].s_pid.clear()
                            robots[robot_id].w_pid.clear()
                            cfg.pid_list[robot_id] = [0, 0]

                            # sys.stdout.write('forward %d %f\n' % (robot_id, 0))
                            robots[robot_id].state = 4
                            robots[robot_id].move_history.clear()
                            robots[robot_id].line_flag = 0
                        else:
                            robots[robot_id].move_history.rotate(-1)
                    else:
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        if (abs(direction - robots[robot_id].toward) <= cfg.PI / 4 and workbench_mode != 1) or (abs(direction - robots[robot_id].toward) <= cfg.PI / 4 and workbench_mode == 1) :
                            cfg.pid_list[robot_id] = [rotate, forward]
                        else:
                            cfg.pid_list[robot_id] = [rotate, 0]
                elif robots[robot_id].state == 7:
                    temp_target = robots[robot_id].move_history[0]
                    end_flag = 1 if robots[robot_id].move_history[1] == (-1, -1) else 0
                    distance = cal_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # direction to target
                    direction = drt_point_x_y(robots[robot_id].x, robots[robot_id].y,  temp_target[0], temp_target[1])
                    # remain_path_len = len(robots[robot_id].move_list_target1)

                    # reach the target workbench
                    if distance <= cfg.DIS_TOLERATE:
                        if end_flag == 1:
                            robots[robot_id].s_pid.clear()
                            robots[robot_id].w_pid.clear()
                            cfg.pid_list[robot_id] = [0, 0]

                            # sys.stdout.write('forward %d %f\n' % (robot_id, 0))
                            robots[robot_id].state = 5
                            robots[robot_id].move_history.clear()
                            robots[robot_id].line_flag = 0
                        else:
                            robots[robot_id].move_history.rotate(-1)
                    else:
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        if (abs(direction - robots[robot_id].toward) <= cfg.PI / 4 and workbench_mode != 1 ) or (abs(direction - robots[robot_id].toward) <= cfg.PI / 4 and workbench_mode == 1 ):
                            cfg.pid_list[robot_id] = [rotate, forward]
                        else:
                            cfg.pid_list[robot_id] = [rotate, 0]
        for i, robot in enumerate(robots):
            # if i not in [1]:
            #     continue
            ## 顶牛判断
            if workbench_mode != 1:
                flag = 0
                if battle_time[i] >= 50:
                    other_robots = robots[0:i] + robots[i:4]
                    for other_robot in other_robots:
                        if battle_time[other_robot.robot_id] >= 50 and (cfg.THING_VALUE[robots[i].take_thing] <= cfg.THING_VALUE[other_robot.take_thing] or other_robot.state in [4, 5, 6, 7]) and cal_point_x_y(robots[i].x, robots[i].y, other_robot.x, other_robot.y) <= 1.5:
                            if robots[i].state == 0:
                                robots[i].state = 4
                            elif robots[i].state == 2:
                                robots[i].state = 5
                            battle_time[i] = 0
                            break
                elif battle_time[i] >= 200:
                    if robots[i].state == 0:
                        robots[i].state = 4
                    elif robots[i].state == 2:
                        robots[i].state = 5
                    battle_time[i] = 0
            ###

            rotate = cfg.pid_list[i][0]
            forward = cfg.pid_list[i][1]
            # log.write(f'robot_id1:{i} {rotate} {forward}\n')
            if workbench_mode == 2:
                if forward != 0:
                    # continue
                    ### 防碰撞
                    # if workbench_mode == 3:
                    #     v, _ = orca(i, robots, cfg.tau, cfg.dt, cfg.pid_list, 3)
                    # else:      
                    v, _ = orca(i, robots, wall_list[i], cfg.tau, cfg.dt, cfg.pid_list)
                    if forward > 0:
                        rotate =  math.atan2(-v[1], v[0])  - robot.toward
                        if rotate > cfg.PI:
                            rotate += -2*cfg.PI
                        elif rotate <= -cfg.PI:
                            rotate += 2*cfg.PI
                        rotate = rotate / cfg.dt
                        forward = sqrt(v[0]**2 + v[1]**2)
                        if cfg.pid_list[i][1] < 0:
                            forward = -forward
            # log.write(f'robot_id2:{i} {rotate} {forward}\n')
            sys.stdout.write('rotate %d %f\n' % (i, rotate))
            sys.stdout.write('forward %d %f\n' % (i, forward))
        ###
        finish()
