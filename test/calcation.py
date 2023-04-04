import math
import sys
from robot import Robot
from workbench import WorkBench
from config import CFG
from queue import Queue
import numpy as np
cfg = CFG()
log = open("path_better.txt", "w")


# cal_function
def cal_x(row: int):
    return 0.25 + row * 0.50
    
def cal_y(col: int):
    # return 0.25 + col * 0.50
    return 50.00 - (0.25 + col * 0.50)

def anti_cal_x(row: float):
    return int((row - 0.25) / 0.50)
    
def anti_cal_y(col: float):
    # return 0.25 + col * 0.50
    return int((50.00 - col - 0.25) / 0.50)

        
def cal_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y: float):
    return math.sqrt((origin_x - target_x) ** 2 + (origin_y - target_y) ** 2)

def drt_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y: float, mode):    
    eps = 1e-6 if mode != 2 else 1e-4
    ans = math.atan2(target_y - origin_y, target_x - origin_x)
    if abs(ans + cfg.PI) < eps:
        return -ans
    return ans

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

def add_more_times_all(workbench, wait_time, go_time, mode=0):
    # if mode == 3:
    #     extra_time = 0
    #     if 1 <= workbench.work_type <= 3:
    #         extra_time += 0
    #     elif wait_time == -1:
    #         extra_time += cfg.MAX_PENTALIY_VALUE
    #     elif workbench.output == 1 or wait_time <= go_time:
    #         extra_time += 0
    #     elif wait_time - go_time > cfg.MAX_WAIT_TIME:
    #         extra_time += cfg.MAX_PENTALIY_VALUE
    #     else:
    #         extra_time += wait_time - go_time
    #     return extra_time
    # else:
    if wait_time == -1 and workbench.output == 1:
        return 0
    if wait_time == -1:
        return cfg.MAX_PENTALIY_VALUE
    extra_time = 0
    if 1 <= workbench.work_type <= 3:
        extra_time += 0
    elif wait_time - go_time > cfg.MAX_WAIT_TIME:
        extra_time += cfg.MAX_PENTALIY_VALUE
    else:
        extra_time += wait_time - go_time
    return extra_time

def choose_target_workbench_list(generate_product, origin_workbench_work_type, choose_mode=1):
    target_workbench_list = []
    if origin_workbench_work_type == 4 or origin_workbench_work_type == 5 or origin_workbench_work_type == 6:
        target_workbench_list = [7, 9]
    elif origin_workbench_work_type == 7:
        target_workbench_list = [8, 9]
    elif origin_workbench_work_type == 1:
        target_workbench_list = [4, 5, 9]
    elif origin_workbench_work_type == 2:
        target_workbench_list = [4, 6, 9]
    elif origin_workbench_work_type == 3:
        target_workbench_list = [5, 6, 9]
    if choose_mode == 2:
        if origin_workbench_work_type == 1: 
            if generate_product[4] - generate_product[5] >= cfg.SUB_MISSION:
                target_workbench_list.pop(0)
            elif generate_product[4] - generate_product[5] <= -cfg.SUB_MISSION:
                target_workbench_list.pop(1)
        if origin_workbench_work_type == 2:
            if generate_product[4] - generate_product[6] >= cfg.SUB_MISSION:
                target_workbench_list.pop(0)
            elif generate_product[4] - generate_product[6] <= -cfg.SUB_MISSION:
                target_workbench_list.pop(1)
        if origin_workbench_work_type == 3:
            if generate_product[5] - generate_product[6] >= cfg.SUB_MISSION:
                target_workbench_list.pop(0)
            elif generate_product[5] - generate_product[6] <= -cfg.SUB_MISSION:
                target_workbench_list.pop(1)
    return target_workbench_list

def get_ava_list(target_workbench_list, workbench_type_num):
    ava_list = []
    for i in target_workbench_list:
        type_num_list = workbench_type_num[i]
        ava_list = ava_list + type_num_list
    return ava_list


def bfs(env_mp, st_pos, is_take_thing):
    q = Queue()
    q.put((st_pos, is_take_thing))
    ans_mp = [[None for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
    vis_mp = [[0 for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
    vis_mp[st_pos[0]][st_pos[1]] = 1
    ans_mp[st_pos[0]][st_pos[1]] = st_pos
    while not q.empty():
        now_pos = q.get()
        for i in range(4):
            nx, ny = now_pos[0][0] + cfg.DIS_NORMAL[i][0], now_pos[0][1] + cfg.DIS_NORMAL[i][1]
            is_taking = now_pos[1]
            if nx < 0 or ny < 0 or nx >= cfg.MAP_SIZE or ny >= cfg.MAP_SIZE or env_mp[nx][ny] == '#' or vis_mp[nx][ny] != 0:
                continue
            if i == 0 and is_taking == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and env_mp[nx][ny - 1] == '#'  and env_mp[nx][ny + 1] == '#')):
                continue 
            if i == 1 and is_taking == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and env_mp[nx][ny - 1] == '#'  and env_mp[nx][ny + 1] == '#')):
                continue 
            if i == 2 and is_taking == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and env_mp[nx + 1][ny] == '#'  and env_mp[nx - 1][ny] == '#')):
                continue 
            if i == 3 and is_taking == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and env_mp[nx + 1][ny] == '#'  and env_mp[nx - 1][ny] == '#')):
                continue 
            vis_mp[nx][ny] = vis_mp[now_pos[0][0]][now_pos[0][1]] + 1
            ans_mp[nx][ny] = now_pos[0]
            # ans_mp[nx][ny] = ans_mp[now_pos[0][0]][now_pos[0][1]]
            # ans_mp[nx][ny].append((nx, ny))
            # ans_mp[nx][ny] = path_better(env_mp, ans_mp[nx][ny], is_take_thing)
            q.put(((nx, ny), is_taking))
    return ans_mp, vis_mp
            
            
def ask_path(ed_pos, ans_mp):
    path = []
    path.append(ed_pos)
    pos = ed_pos
    log.write(f"{ed_pos}\n")
    log.write(f"{ans_mp}\n")
    log.write(f"---------\n")
    nx, ny = pos[0], pos[1]
    while ans_mp[nx][ny] != pos:
        pos = ans_mp[nx][ny]
        nx, ny = pos[0], pos[1]
        path.append(pos)
    path.reverse()
    return path