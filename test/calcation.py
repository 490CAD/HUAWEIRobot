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
def cal_x(row: float):
    return 0.25 + row * 0.50
    
def cal_y(col: float):
    # return 0.25 + col * 0.50
    return 50.00 - (0.25 + col * 0.50)

def anti_cal_x(row: float):
    return int((row - 0.25) / 0.50)
    
def anti_cal_y(col: float):
    # return 0.25 + col * 0.50
    return int((50.00 - col - 0.25) / 0.50)

        
def cal_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y: float):
    return math.sqrt((origin_x - target_x) ** 2 + (origin_y - target_y) ** 2)

def drt_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y: float, mode=0):    
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
    # log.write(f"{len(env_mp), len(env_mp[0])}\n")
    ans_mp = [[None for i in range(cfg.MAP_SIZE_2)] for j in range(cfg.MAP_SIZE_2)]
    vis_mp = [[0 for i in range(cfg.MAP_SIZE_2)] for j in range(cfg.MAP_SIZE_2)]
    vis_mp[st_pos[0]][st_pos[1]] = 1
    ans_mp[st_pos[0]][st_pos[1]] = st_pos
    while not q.empty():
        now_pos = q.get()
        for i in range(8):
            nx, ny = now_pos[0][0] + cfg.DIS_HIGHER[i][0], now_pos[0][1] + cfg.DIS_HIGHER[i][1]
            # nx, ny = now_pos[0][0] + cfg.DIS_NORMAL[i][0], now_pos[0][1] + cfg.DIS_NORMAL[i][1]
            is_taking = now_pos[1]
            if nx < 0 or ny < 0 or nx >= cfg.MAP_SIZE_2 or ny >= cfg.MAP_SIZE_2 or vis_mp[nx][ny] != 0 or env_mp[nx][ny] == '#':
                continue
            if check_points(env_mp, nx, ny ,is_taking) == 0:
                continue
                
            vis_mp[nx][ny] = vis_mp[now_pos[0][0]][now_pos[0][1]] + 1
            ans_mp[nx][ny] = now_pos[0]
            q.put(((nx, ny), is_taking))
        # for i in range(4):
        #     nx, ny = now_pos[0][0] + cfg.HALF_DIS_2[i][0], now_pos[0][1] + cfg.HALF_DIS_2[i][2]
        #     if nx < 0 or ny < 0 or nx >= cfg.MAP_SIZE or ny >= cfg.MAP_SIZE or env_mp[nx][ny] == '#' or vis_mp[nx][ny] != 0:
        #         continue
            
            
    return ans_mp, vis_mp
            
            
def ask_path(ed_pos, ans_mp, env_mp):
    path = []
    path.append(ed_pos)
    pos = ed_pos
    nx, ny = pos[0], pos[1]
    log.write(f"{ans_mp[nx][ny]}\n")
    log.write(f"{env_mp[nx][ny]}\n")
    log.write(f"{nx, ny}\n")
    while ans_mp[nx][ny] != pos:
        pos = ans_mp[nx][ny]
        nx, ny = pos[0], pos[1]
        
        log.write(f"{nx, ny}\n")
        path.append(pos)
    path.reverse()
    path = path_better(env_mp, path, 1)
    log.write(f"{path}\n")
    for i in range(len(path)):
        path[i] = (cal_x(path[i][1] / 2), cal_y(path[i][0] / 2))
    log.write(f"{path}\n")
    log.write(f"-----------------\n")
    return path

def check_one_line(point1, point2, point3):
    # all tuple
    # y2 - y1 / x2 - x1 = y3 - y1 / x3 - x1
    if (point2[1] - point1[1]) * (point3[0] - point1[0]) == (point3[1] - point1[1]) * (point2[0] - point1[0]):
        return 1
    return 0

def check_points(env_mp, nx, ny, is_take_thing):
    if env_mp[nx][ny] == '#':
        return 0
            
    ny1, ny2 = max(0, ny - 1), min(cfg.MAP_SIZE_2 - 1, ny + 1)
    nx1, nx2 = max(0, nx - 1), min(cfg.MAP_SIZE_2 - 1, nx + 1)
    if env_mp[nx][ny1] == '#' or env_mp[nx][ny2] == '#' or env_mp[nx1][ny] == '#' or env_mp[nx2][ny] == '#':
        return 0
    if env_mp[nx1][ny1] == '#' or env_mp[nx1][ny2] == '#' or env_mp[nx2][ny1] == '#' or env_mp[nx2][ny2] == '#':
        return 0
    if is_take_thing == 1:
        nx3, nx4 = max(0, nx - 2), min(cfg.MAP_SIZE_2 - 1, nx + 2)
        ny3, ny4 = max(0, ny - 2), min(cfg.MAP_SIZE_2 - 1, ny + 2)
        # log.write(f"{nx1, nx2, nx3, nx4}\n")
        # log.write(f"{ny1, ny2, ny3, ny4}\n")
        if env_mp[nx3][ny] == '#':
            return 0
        if env_mp[nx][ny3] == '#' or env_mp[nx][ny4] == '#':
            return 0
        if env_mp[nx4][ny] == '#':
            return 0
        if env_mp[nx3][ny1] == '#' or env_mp[nx3][ny2] == '#' or env_mp[nx4][ny1] == '#' or env_mp[nx4][ny2] == '#' or env_mp[nx1][ny3] == '#' or env_mp[nx1][ny4] == '#' or env_mp[nx2][ny3] == '#' or env_mp[nx2][ny4] == '#':
            return 0
    return 1

def ignore_now_point(env_mp, point1, point2, point3, is_take_thing):
    # x_1 == x_3, y_1 == y_3
    min_x, max_x = min(point1[0], point3[0]), max(point1[0], point3[0])
    min_y, max_y = min(point1[1], point3[1]), max(point1[1], point3[1])
    if point1[0] == point3[0]:
        for i in range(min_y, max_y + 1):
            nx, ny = min_x, i
            if check_points(env_mp, nx, ny, is_take_thing) == 0:
                return 0
    elif point1[1] == point3[1]:
        for i in range(min_x, max_x + 1):
            nx, ny = i, min_y
            
            if check_points(env_mp, nx, ny, is_take_thing) == 0:
                return 0
    else:
        # return 0
        # y = kx +b
        # y_1 = kx_1 + b; y_2 = kx_2 +b; k = (y_2 - y_1) /(x_2 - x_1); b=y_2 - x_2*k
        k = (point3[1] - point1[1]) / (point3[0] - point1[0])
        b = point3[1] - point3[0] * k
        # log.write(f"{point1, point3}\n")
        for i in range(min_x, max_x):
            nx = i
            ny = (k * nx + b)

            int_ny = int(ny)
            # if int_ny == ny:

            #     if check_points(env_mp, nx, int_ny, is_take_thing) == 0:
            #         return 0
            #     # if env_mp[nx][int_ny - 1] == '#' or env_mp[nx][int_ny] =='#' or env_mp[nx][int_ny + 1] == '#':
            #     #     continue
        # else:
            ny = int_ny
            # log.write(f"{nx, ny}\n")
            # TODO: 可能会存在问题  # 确实有问题
            if check_points(env_mp, nx, ny, is_take_thing) == 0:
                return 0
            if check_points(env_mp, nx, ny + 1, is_take_thing) == 0:
                return 0
            if check_points(env_mp, nx, ny - 1, is_take_thing) == 0:
                return 0

            # ny += 1
            # if env_mp[nx][ny] == '#':
            #     return 0
            # if is_take_thing == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and (env_mp[nx][ny - 1] == '#'  or env_mp[nx][ny + 1] == '#'))):
            #     return 0
            # if is_take_thing == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and (env_mp[nx + 1][ny] == '#'  or env_mp[nx - 1][ny] == '#'))):
            #     return 0
            # ny -= 2
            # if env_mp[nx][ny] == '#':
            #     return 0
            # if is_take_thing == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and (env_mp[nx][ny - 1] == '#'  or env_mp[nx][ny + 1] == '#'))):
            #     return 0
            # if is_take_thing == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and (env_mp[nx + 1][ny] == '#'  or env_mp[nx - 1][ny] == '#'))):
            #     return 0
        # log.write("-------------------\n")
    
    return 1

def path_better(env_mp, path_list, is_take_thing):
    if len(path_list) <= 1:
        return path_list
    # log.write(f"{path_list}\n")
    path_len = len(path_list)
    new_path = []
    pre_point = path_list[0]
    new_path.append(pre_point)
    shr_point = path_list[1]
    nxt_point = shr_point
    for pos in range(2, path_len):
        now_point = path_list[pos]
        if check_one_line(pre_point, shr_point, now_point) == 1:
            shr_point = now_point
            nxt_point = shr_point
            continue
        else:
            if ignore_now_point(env_mp, pre_point, shr_point, now_point, is_take_thing) == 1:
                nxt_point = now_point
                continue
            else:
                new_path.append(nxt_point)
                pre_point = nxt_point
                if pos + 1 >= path_len:
                    break
                else:
                    shr_point = nxt_point = path_list[pos + 1]
    new_path.append(nxt_point)
    # log.write(f"{new_path}\n")
    return new_path