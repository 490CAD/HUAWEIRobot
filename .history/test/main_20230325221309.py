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
from robot import Robot
from workbench import WorkBench
from config import CFG
from calcation import *
# import argparse
from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp
from numpy import array, rint, linspace, pi, cos, sin, sqrt

# hyperparameters
cfg = CFG()
log = open("log.txt", "a")
# global needs

# map1 43
# map2 25
# map3 50
# map4 18
"""
    high_level_workbench_list: 类型为4567的工作台
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
high_level_workbench_list = []
useful_workbench_list = []
env_mp, DIS_MP = None, None
workbench_ids = 0
workbenchs, robots = [], []
workbench_type_num = [[] for i in range(10)]
workbench_minest_sell = []
generate_product = {4:0, 5:0, 6:0}
workbench_mode = 0

# def parse_args():
#     parse = argparse.ArgumentParser(description='Calculate cylinder volume')  # 2、创建参数对象
#     parse.add_argument('--wait_time', type=int, help='wait time')  # 3、往参数对象添加参数
#     args = parse.parse_args()  # 4、解析参数对象获得解析对象
#     return args
def get_price_by_look_further(free_robots):
    robot_id, target0_id, target1_id, best_val_time = -1, -1, -1, 0.0
    workbench_list = useful_workbench_list
    target2_id = -1
    for id in free_robots:
        robot = robots[id]
        for target0 in workbench_list:
            target0_workbench = workbenchs[target0]
            if target0_workbench.is_targeted_flag[0] == 1 or (target0_workbench.output != 1 and target0_workbench.work_type in cfg.HIGH_LEVEL_WORKBENCH and target0_workbench.remain_time == -1):
                continue

            target_workbench_list = choose_target_workbench_list(generate_product, target0_workbench.work_type, 2)
            ava_list = get_ava_list(target_workbench_list, workbench_type_num)

            for target1 in ava_list:

                target1_workbench = workbenchs[target1]
                if target1_workbench.work_type in  cfg.HIGH_LEVEL_WORKBENCH:
                    if  target1_workbench.is_targeted_flag[target0_workbench.work_type] == 1 or ((1 << target0_workbench.work_type) & target1_workbench.origin_thing) != 0:
                        continue

                target0_target1_dis = DIS_MP[target0][target1]
                robot_target0_dis = cal_point_x_y(robot.x, robot.y, target0_workbench.x, target0_workbench.y)
                all_dis = robot_target0_dis + target0_target1_dis
                wait_time = target0_workbench.remain_time
                robot_target0_time = robot_target0_dis * 50 / 6
                all_time = all_dis * 50 / 6 + add_more_times_all(target0_workbench, wait_time, robot_target0_time)
                temp_val = cfg.THING_VALUE[target0_workbench.work_type]
                temp_val_time = temp_val / all_time

                next_time = 0
                if target1_workbench.work_type in cfg.HIGH_LEVEL_WORKBENCH:
                    if workbench_minest_sell[target1][0] == -1:
                        next_time = DIS_MP[target1][workbench_minest_sell[target1][1]] * 50 / 6
                    elif workbench_minest_sell[target1][1] == -1:
                        next_time = DIS_MP[target1][workbench_minest_sell[target1][0]] * 50 / 6
                    else:
                        next_time = min(DIS_MP[target1][workbench_minest_sell[target1][1]], DIS_MP[target1][workbench_minest_sell[target1][0]]) * 50 / 6

                if target1_workbench.work_type == 7:
                    if target1_workbench.origin_thing == 0:
                        next_time = (next_time + 1000) * 3
                    elif ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 112:
                        next_time = (next_time + 1000)
                    else:
                        next_time = (next_time + 1000) * 2
                elif target1_workbench.work_type in [4, 5, 6]:
                    if target1_workbench.origin_thing == 0:
                        next_time = (next_time + 500) * 2
                    else:
                        next_time = (next_time + 500)
                if target1_workbench.work_type not in [8, 9] and all_time <= cfg.MAX_PENTALIY_VALUE and (target1_workbench.work_type == 7 and ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 112):
                    temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] # / next_time

                if temp_val_time > best_val_time:
                    robot_id, target0_id, target1_id = id, target0, target1
                    best_val_time = temp_val_time

    # robots[robot_id].value = best_val_time
    return robot_id, target0_id, target1_id, target2_id

def get_price_by_targets(free_robots, work_mode, frame_id):
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
    global workbench_mode

    robot_id, target0_id, target1_id, best_val_time = -1, -1, -1, 0.0
    workbench_list = useful_workbench_list
    # if(workbench_ids in [50] and (9000 - frame_id < 300)):
    #     workbench_list = workbench_type_num[6]
    for id in free_robots:
        robot = robots[id]
        for target0 in workbench_list:
            target0_workbench = workbenchs[target0]
            
            if workbench_mode == 3:
                if target0_workbench.work_type == 6 and id in [2, 3]:
                    continue
                if target0_workbench.work_type == 5 and id in [0, 1]:
                    continue
                
            if target0_workbench.is_targeted_flag[0] == 1 or (target0_workbench.output != 1 and target0_workbench.work_type in cfg.HIGH_LEVEL_WORKBENCH and target0_workbench.remain_time == -1):
                continue
            
            if workbench_mode == 1 and (target0_workbench.output != 1 and target0_workbench.work_type in cfg.HIGH_LEVEL_WORKBENCH and target0_workbench.remain_time >= 50):
                continue
            if workbench_mode == 1 and target0_workbench.work_type in [4, 5, 6]:
                ava_list = [11, 22, 15, 17, 10, 12, 21, 23]
            if workbench_mode == 3:
                if target0_workbench.work_type in [4]:
                    continue
                if target0_workbench.work_type in [2]:
                    target_workbench_list = [6]
                elif target0_workbench.work_type in [1]:
                    target_workbench_list = [5]
                elif target0_workbench.work_type in [3]:
                    target_workbench_list = [5, 6]
                else:
                    target_workbench_list = [9]
                ava_list = get_ava_list(target_workbench_list, workbench_type_num)
            else:
                target_workbench_list = choose_target_workbench_list(generate_product, target0_workbench.work_type, work_mode)
                ava_list = get_ava_list(target_workbench_list, workbench_type_num)

            for target1 in ava_list:
                target1_workbench = workbenchs[target1]
                if workbench_mode == 3:
                    if target1_workbench.work_type == 6 and id in [2, 3]:
                        continue
                    if target1_workbench.work_type == 5 and id in [0, 1]:
                        continue
                    
                if target1_workbench.work_type == 9 and target0_workbench.work_type in [4, 5, 6] and workbench_mode == 1:
                    continue
                if target1_workbench.work_type in  cfg.HIGH_LEVEL_WORKBENCH:
                    if  target1_workbench.is_targeted_flag[target0_workbench.work_type] == 1 or ((1 << target0_workbench.work_type) & target1_workbench.origin_thing) != 0:
                        continue

                target0_target1_dis = DIS_MP[target0][target1]
                robot_target0_dis = cal_point_x_y(robot.x, robot.y, target0_workbench.x, target0_workbench.y)
                all_dis = robot_target0_dis + target0_target1_dis
                wait_time = target0_workbench.remain_time
                robot_target0_time = robot_target0_dis * 50 / 6
                all_time = all_dis * 50 / 6 + add_more_times_all(target0_workbench, wait_time, robot_target0_time, workbench_mode)
                temp_val = cfg.THING_VALUE[target0_workbench.work_type]
                temp_val_time = temp_val / all_time
                next_time = 0
                if target1_workbench.work_type in cfg.HIGH_LEVEL_WORKBENCH:
                    if workbench_minest_sell[target1][0] == -1:
                        next_time = DIS_MP[target1][workbench_minest_sell[target1][1]] * 50 / 6
                    elif workbench_minest_sell[target1][1] == -1:
                        next_time = DIS_MP[target1][workbench_minest_sell[target1][0]] * 50 / 6
                    else:
                        next_time = min(DIS_MP[target1][workbench_minest_sell[target1][1]], DIS_MP[target1][workbench_minest_sell[target1][0]]) * 50 / 6

                if target1_workbench.work_type == 7:
                    if target1_workbench.origin_thing == 0:
                        next_time = (next_time + 1000) * 3
                    elif ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 112:
                        next_time = (next_time + 1000)
                    else:
                        next_time = (next_time + 1000) * 2
                elif target1_workbench.work_type in [4, 5, 6]:
                    if target1_workbench.origin_thing == 0:
                        next_time = (next_time + 500) * 2
                    else:
                        next_time = (next_time + 500)
                if target1_workbench.work_type not in [8, 9] and all_time <= cfg.MAX_PENTALIY_VALUE:
                    if (target1_workbench.work_type == 7 and ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 112):
                        temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] # / next_time
                    if (target1_workbench.work_type == 4 and ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 6):
                        temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] / next_time
                    if (target1_workbench.work_type == 5 and ((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 10):
                        temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] / next_time
                    if (target1_workbench.work_type == 6 and (((1 << target0_workbench.work_type) | target1_workbench.origin_thing) == 12)):
                        temp_val_time += cfg.THING_VALUE[target1_workbench.work_type] / next_time
                if temp_val_time > best_val_time:
                    robot_id, target0_id, target1_id = id, target0, target1
                    best_val_time = temp_val_time
    robots[robot_id].value = best_val_time
    return robot_id, target0_id, target1_id  

# store something 
def find_nearest_target_sell(x, y, target_workbench_list, take_thing):
    ava_list = get_ava_list(target_workbench_list, workbench_type_num)
            
    target_workbench_id = -1
    target_workbench_distance = 300
    for i in ava_list:
        if workbenchs[i].work_type in cfg.HIGH_LEVEL_WORKBENCH:
            if  workbenchs[i].is_targeted_flag[take_thing] == 1 or ((1 << take_thing) & workbenchs[i].origin_thing) != 0:
                continue
        R_W_distance = cal_point_x_y(x, y, workbenchs[i].x, workbenchs[i].y)
        if target_workbench_distance > R_W_distance:
            target_workbench_id, target_workbench_distance = i, R_W_distance
    return target_workbench_id

def get_price_by_time(free_robots):
    """
        robot_id -> 执行任务的机器人, target0_id -> 去买的工作台, target1_id ->去卖的工作台
        best_val_time -> max(盈利 / 时间(robot->target0->target1))
        workbench_list -> 工作台类型为1-7的工作台
        robot_dis -> robot到target0的距离
        target_workbench_list -> 可选的target1的工作台类型列表(target1通过最近距离来找)[接口:find_nearest_target_sell]
        all_dis -> robot_dis + target0到target1的距离
        go_time -> robot_dis花费时间
        wait_time -> 工作台target0生产物品所需要的剩余时间
        all_time -> 整个过程的时间
    """
    robot_id, target0_id, target1_id = -1, -1, -1
    best_val_time = 0.0
    workbench_list = useful_workbench_list
    for robot in free_robots:
        for target0 in workbench_list:
            if workbenchs[target0].is_targeted_flag[0] == 1:
                continue
            robot_dis = cal_point_x_y(robots[robot].x, robots[robot].y, workbenchs[target0].x, workbenchs[target0].y)
            target_workbench_list = choose_target_workbench_list(generate_product, workbenchs[target0].work_type)
            target1 = find_nearest_target_sell(workbenchs[target0].x ,workbenchs[target0].y, target_workbench_list, workbenchs[target0].work_type)
            if target1 == -1:
                continue
            all_dis = robot_dis + DIS_MP[target0][target1]
            go_time = robot_dis * 50 / 6.0
            wait_time = max(workbenchs[target0].remain_time, 0)
            all_time = all_dis * 50 / 6.0 + add_more_times_all(workbenchs[target0], wait_time, go_time)
            
            temp_val = cfg.THING_VALUE[workbenchs[target0].work_type]
            temp_val_time = temp_val / all_time
            if temp_val_time > best_val_time:
                robot_id, target0_id, target1_id = robot, target0, target1
                best_val_time = temp_val_time
    # robots[robot_id].value = best_val_time
    return robot_id, target0_id, target1_id   

def map_init():
    global workbench_ids
    global workbench_mode
    robot_ids = 0
    for row in range(cfg.MAP_SIZE):
        for col in range(cfg.MAP_SIZE):
            if '1' <= env_mp[row][col] <= '9':
                workbench_type = int(env_mp[row][col])
                workbenchs.append(WorkBench(workbench_ids))
                workbench_type_num[workbench_type].append(workbench_ids)
                workbench_minest_sell.append([])
                workbenchs[workbench_ids].work_type = workbench_type
                workbenchs[workbench_ids].x, workbenchs[workbench_ids].y = cal_x(col), cal_y(row)
                workbench_ids += 1
            if env_mp[row][col] == 'A':
                robots.append(Robot(robot_ids))
                robot_ids += 1

    for workbench_a in range(0, workbench_ids):
        for workbench_b in range(workbench_a + 1, workbench_ids):
            DIS_MP[workbench_a][workbench_b] = DIS_MP[workbench_b][workbench_a] = cal_point_x_y(workbenchs[workbench_a].x, workbenchs[workbench_a].y, workbenchs[workbench_b].x, workbenchs[workbench_b].y)

    for workbench_type in cfg.HIGH_LEVEL_WORKBENCH:
        for workbench in workbench_type_num[workbench_type]:
            high_level_workbench_list.append(workbench)

    for workbench_type in cfg.USEFUL_WORKBENCH:
        for workbench in workbench_type_num[workbench_type]:
            useful_workbench_list.append(workbench)
    # log.write(f'{useful_workbench_list}\n')

    for workbench_a in range(0, workbench_ids):
        target_workbench_type = choose_target_workbench_list(generate_product, workbenchs[workbench_a].work_type)
        cnt = -1
        for type in target_workbench_type:
            workbench_minest_sell[workbench_a].append(-1)
            cnt += 1
            for workbench_b in workbench_type_num[type]:
                if workbench_minest_sell[workbench_a][cnt] == -1 or DIS_MP[workbench_a][workbench_b] < DIS_MP[workbench_a][workbench_minest_sell[workbench_a][cnt]]:
                    workbench_minest_sell[workbench_a][cnt] = workbench_b
    if workbench_ids in [43]:
        for i in range(workbench_ids):
            if workbenchs[i].work_type in [4, 5 ,6]:
                for j in range(workbench_ids):
                    if workbenchs[j].work_type == 7:
                        DIS_MP[i][j] = DIS_MP[j][i] = 0 
                        
    if workbench_ids in [50]:
        workbench_type_num[4] = sorted(workbench_type_num[4], key=functools.cmp_to_key(map3cmp))
        workbench_type_num[5] = sorted(workbench_type_num[5], key=functools.cmp_to_key(map3cmp))
        workbench_type_num[6] = sorted(workbench_type_num[6], key=functools.cmp_to_key(map3cmp))
        # workbench_type_num[6] = workbench_type_num[6][0:2]

    if workbench_ids == 50:
        workbench_mode = 3
        cfg.MAX_WAIT_TIME = 30
    elif workbench_ids == 43:
        workbench_mode = 1
        cfg.MAX_WAIT_TIME = 50
    
def map3cmp(x, y):
    x_dis = cal_point_x_y(workbenchs[x].x, workbenchs[x].y, workbenchs[workbench_type_num[9][0]].x, workbenchs[workbench_type_num[9][0]].y)
    y_dis = cal_point_x_y(workbenchs[y].x, workbenchs[y].y, workbenchs[workbench_type_num[9][0]].x, workbenchs[workbench_type_num[9][0]].y)
    if x_dis < y_dis:
        return -1
    if x_dis > y_dis:
        return 1
    return 0

# Main
if __name__ == '__main__':
    # input env_map
    env_mp, DIS_MP = read_map(), [[50.0 * 50.0 for j in range(50)] for i in range(50)]
    map_init()
    finish()
    # start working
    # args = parse_args()
    # log.write(f"{args}\n")
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

        # 分配任务
        free_robots = find_free_robot(robots)
        # free_jobs = find_free_job(workbenchs)
        # log.write(f'--------------------------------{frame_id}\n')
        # log.write(f'{free_robots}\n')
        # log.write(f'0 {robots[0].target_workbench_ids}\n')
        # log.write(f'1 {robots[1].target_workbench_ids}\n')
        # log.write(f'2 {robots[2].target_workbench_ids}\n')
        # log.write(f'3 {robots[3].target_workbench_ids}\n')
        # log.write(f'----------------\n')

        for i in range(len(free_robots)):
            employ_robot, target0, target1 = get_price_by_targets(free_robots, 2, frame_id)
            if employ_robot == -1:
                employ_robot, target0, target1 = get_price_by_targets(free_robots, 1, frame_id)
            # employ_robot, target0, target1 = get_price_by_look_further(free_robots)
                # employ_robot, target0, target1 = get_price_by_time(free_robots)
            if employ_robot != -1:
                robots[employ_robot].target_workbench_ids[0] = target0
                robots[employ_robot].target_workbench_ids[1] = target1
                # if workbenchs[target0].work_type not in [1, 2, 3]:
                workbenchs[robots[employ_robot].target_workbench_ids[0]].is_targeted_flag[0] = 1
                workbenchs[robots[employ_robot].target_workbench_ids[1]].is_targeted_flag[workbenchs[robots[employ_robot].target_workbench_ids[0]].work_type] = 1
                # if workbenchs[robots[employ_robot].target_workbench_ids[0]].work_type in [1, 2, 3]:
                #     workbenchs[robots[employ_robot].target_workbench_ids[0]].is_targeted_flag[0] = 0
                free_robots.remove(employ_robot)

        line = sys.stdin.readline()

        # do some operation
        sys.stdout.write('%d\n' % frame_id)

        # if frame_id <=100:
        #     direction = -cfg.PI / 4
        #     distance = 10
        #     rotate, forward = robots[1].move_to_target(direction, distance)
        #     cfg.pid_list[1] = [rotate, forward]

        for robot_id in range(cfg.ROBOT_NUM):
            rotate, forward = None, None
            if robots[robot_id].target_workbench_ids[0] == -1:
                continue
            if robots[robot_id].work_frame <= frame_id:
                if robots[robot_id].state == 0:
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
                        robots[robot_id].state = 1
                    else:
                        # rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        # cfg.pid_list[robot_id] = [rotate, forward]
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        if abs(direction - robots[robot_id].toward) <= cfg.PI / 2:
                            cfg.pid_list[robot_id] = [rotate, forward]
                        else:
                            cfg.pid_list[robot_id] = [rotate, 0]
                        # cfg.pid_list[robot_id] = [rotate, forward]
                elif robots[robot_id].state == 1:
                    # buy
                    if workbenchs[robots[robot_id].target_workbench_ids[0]].output == 1 and robots[robot_id].work_space == robots[robot_id].target_workbench_ids[0]:
                        if DIS_MP[robots[robot_id].target_workbench_ids[0]][robots[robot_id].target_workbench_ids[1]] / 6 > (9000 - frame_id) / 50 - 1:
                            # go to 8,9
                            continue
                        sys.stdout.write('buy %d\n' % robot_id)
                        workbenchs[robots[robot_id].target_workbench_ids[0]].is_targeted_flag[0] = 0
                        robots[robot_id].state = 2
                    else:
                        robots[robot_id].state = 0
                elif robots[robot_id].state == 2:
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
                        robots[robot_id].state = 3
                    else:
                        # rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        # cfg.pid_list[robot_id] = [rotate, forward]
                        rotate, forward = robots[robot_id].move_to_target(direction, distance)
                        if abs(direction - robots[robot_id].toward) <= cfg.PI / 2:
                            cfg.pid_list[robot_id] = [rotate, forward]
                        else:
                            cfg.pid_list[robot_id] = [rotate, 0]
                        # cfg.pid_list[robot_id] = [rotate, forward]

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
                        robots[robot_id].state = 2
        # log.write(f'{robots[3].x}, {robots[3].y}\n')
        # 32.75 + 5
        # distance = cal_point_x_y(robots[1].x, robots[1].y, 12.75 - 5, 35.75)
        #             # direction to target
        # direction = drt_point_x_y(robots[1].x, robots[1].y, 12.75 - 5, 35.75)
        # log.write(f'{robots[0].x} {robots[0].y}\n')
        # rotate, forward = robots[1].move_to_target(direction, distance)
        # rotate, forward =0, -2
        # cfg.pid_list= [[0, 0],[0, 0],[0, 0],[0, 0]]
        # cfg.pid_list[1]= [rotate, forward]
        # robots[0].value = 100
        # robots[1].value = 0
        ### 防碰撞检测与预防
        # for i, robot in enumerate(robots):
        #     # if i not in [1]:
        #     #     continue
        #     if cfg.pid_list[i][0] == 0:
        #         continue
        #     rotate = cfg.pid_list[i][0]
        #     forward = cfg.pid_list[i][1]
        #     v, _ = orca(i, robots, cfg.tau, cfg.dt, cfg.pid_list)
        #     if cfg.pid_list[i][1] >= 0:
        #         rotate =  math.atan2(-v[1], v[0])  - robot.toward
        #         if rotate > cfg.PI:
        #             rotate += -2*cfg.PI
        #         elif rotate <= -cfg.PI:
        #             rotate += 2*cfg.PI
        #         rotate = rotate / cfg.dt
        #         forward = sqrt(v[0]**2 + v[1]**2)
        #         if cfg.pid_list[i][1] < 0:
        #             forward = -forward
        #         # rotate = -rotate
        #     # log.write(f'rotate{rotate} forward{forward}\n\n')
        ### 防碰撞检测与预防
        for i, robot in enumerate(robots):
            # if i not in [1]:
            #     continue
            rotate = cfg.pid_list[i][0]
            forward = cfg.pid_list[i][1]
            if cfg.pid_list[i][1] != 0:
                # continue
                ### 防碰撞
                if workbench_mode == 3:
                    v, _ = orca(i, robots, cfg.tau, cfg.dt, cfg.pid_list, 3)
                else:      
                    v, _ = orca(i, robots, cfg.tau, cfg.dt, cfg.pid_list)
                if cfg.pid_list[i][1] >= 0:
                    rotate =  math.atan2(-v[1], v[0])  - robot.toward
                    if rotate > cfg.PI:
                        rotate += -2*cfg.PI
                    elif rotate <= -cfg.PI:
                        rotate += 2*cfg.PI
                    rotate = rotate / cfg.dt
                    forward = sqrt(v[0]**2 + v[1]**2)
                    if cfg.pid_list[i][1] < 0:
                        forward = -forward
            ##
            sys.stdout.write('rotate %d %f\n' % (i, rotate))
            sys.stdout.write('forward %d %f\n' % (i, forward))
        ###
        # log.write(f'----------------------------------------------------------------\n')
        finish()
