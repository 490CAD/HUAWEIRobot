import math
import sys
from robot import Robot
from workbench import WorkBench
from config import CFG

cfg = CFG()


# cal_function
def cal_x(row: int):
    return 0.25 + row * 0.50
    
def cal_y(col: int):
    # return 0.25 + col * 0.50
    return 50.00 - (0.25 + col * 0.50)
        
def cal_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y: float):
    return math.sqrt((origin_x - target_x) ** 2 + (origin_y - target_y) ** 2)

def drt_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y: float):
    return math.atan2(target_y - origin_y, target_x - origin_x)

# Input and Output Functions 
def read_map():
    env_mp, row_cnt = [[] for i in range(cfg.MAP_SIZE)], 0
    while True:
        line = input()
        if line == "OK":
            break
        for ch in line:
            env_mp[row_cnt].append(ch)
        row_cnt += 1
    return env_mp

        
def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


# Useful function
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

def add_more_times_all(workbench, wait_time, go_time):
    extra_time = 0
    if 1 <= workbench.work_type <= 3:
        extra_time += 0
    elif wait_time == -1:
        extra_time += cfg.MAX_PENTALIY_VALUE
    elif workbench.output == 1 or wait_time <= go_time:
        extra_time += 0
    elif wait_time - go_time > cfg.MAX_WAIT_TIME:
        extra_time += cfg.MAX_PENTALIY_VALUE
    else:
        extra_time += workbench.remain_time - go_time
    return extra_time