import math
import sys
from robot import Robot
from workbench import WorkBench
from config import CFG
from queue import Queue
import numpy as np
cfg = CFG()
# log = open("path_better.txt", "w")


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
    q.put((st_pos, is_take_thing, (-1, -1)))
    ans_mp = [[[] for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
    vis_mp = [[0 for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
    vis_mp[st_pos[0]][st_pos[1]] = 1
    ans_mp[st_pos[0]][st_pos[1]] = [st_pos]
    while not q.empty():
        now_pos = q.get()
        for i in range(4):
            nx, ny = now_pos[0][0] + cfg.DIS_NORMAL[i][0], now_pos[0][1] + cfg.DIS_NORMAL[i][1]
            is_taking = now_pos[1]
            if nx < 0 or ny < 0 or nx >= cfg.MAP_SIZE or ny >= cfg.MAP_SIZE or env_mp[nx][ny] == '#' or vis_mp[nx][ny] == 1:
                continue
            if i == 0 and is_taking == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and env_mp[nx][ny - 1] == '#'  and env_mp[nx][ny + 1] == '#')):
                continue 
            if i == 1 and is_taking == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and env_mp[nx][ny - 1] == '#'  and env_mp[nx][ny + 1] == '#')):
                continue 
            if i == 2 and is_taking == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and env_mp[nx + 1][ny] == '#'  and env_mp[nx - 1][ny] == '#')):
                continue 
            if i == 3 and is_taking == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and env_mp[nx + 1][ny] == '#'  and env_mp[nx - 1][ny] == '#')):
                continue 
            temp_ans = []
            for point in ans_mp[now_pos[0][0]][now_pos[0][1]]:
                temp_ans.append(point)
            temp_ans.append((nx, ny))
            vis_mp[nx][ny] = 1
            ans_mp[nx][ny] = temp_ans
            # ans_mp[nx][ny] = ans_mp[now_pos[0][0]][now_pos[0][1]]
            # ans_mp[nx][ny].append((nx, ny))
            # ans_mp[nx][ny] = path_better(env_mp, ans_mp[nx][ny], is_take_thing)
            q.put(((nx, ny), is_taking, (now_pos[0])))
    return ans_mp

def bfs_np(env_mp, st_pos, is_take_thing):
    # [ [x,y]  [is_take_thing, is_take_thing] [-1, -1] ]
    q = np.array([[np.array(st_pos), np.full(2, is_take_thing), np.array([-1, -1])]])

    ans_mp = np.full((cfg.MAP_SIZE, cfg.MAP_SIZE, cfg.MAX_ANS_MP_SIZE, 2), -1, dtype = int)
    # ans_mp = np.array([np.array([np.array([]) for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE))])
    vis_mp = np.zeros((cfg.MAP_SIZE, cfg.MAP_SIZE), dtype = int)

    # ans_mp1 = [[[] for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
    # vis_mp1 = [[0 for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
    # log.write(f'ans_mp:\n{ans_mp}\nans_mp1:\n{ans_mp1}\n')
    # log.write(f'vis_mp:\n{vis_mp}\nvis_mp1:\n{vis_mp1}\n')
    
    # index_mp: 用于标记ans_mp更新到了第几步
    index_mp = np.zeros((cfg.MAP_SIZE, cfg.MAP_SIZE), dtype = int)
    vis_mp[st_pos[0]][st_pos[1]] = 1
    ans_mp[st_pos[0]][st_pos[1]][index_mp[st_pos[0]][st_pos[1]]] = np.array(st_pos)
    # log.write(f'{index_mp}\n')
    index_mp[st_pos[0]][st_pos[1]] += 1

    while q.size != 0:
        now_pos = q[0]
        # log.write(f'q{q}\n')
        q = np.delete(q, 0, axis=0)

        for i in range(4):
            # log.write(f'{now_pos}\n')
            nx, ny = now_pos[0][0] + cfg.DIS_NORMAL[i][0], now_pos[0][1] + cfg.DIS_NORMAL[i][1]
            is_taking = now_pos[1][0]
            if nx < 0 or ny < 0 or nx >= cfg.MAP_SIZE or ny >= cfg.MAP_SIZE or env_mp[nx][ny] == '#' or vis_mp[nx][ny] == 1:
                continue
            if i == 0 and is_taking == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and env_mp[nx][ny - 1] == '#'  and env_mp[nx][ny + 1] == '#')):
                continue 
            if i == 1 and is_taking == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and env_mp[nx][ny - 1] == '#'  and env_mp[nx][ny + 1] == '#')):
                continue 
            if i == 2 and is_taking == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and env_mp[nx + 1][ny] == '#'  and env_mp[nx - 1][ny] == '#')):
                continue 
            if i == 3 and is_taking == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and env_mp[nx + 1][ny] == '#'  and env_mp[nx - 1][ny] == '#')):
                continue 
            # temp_ans = []
            # for point in ans_mp[now_pos[0][0]][now_pos[0][1]]:
            #     temp_ans.append(point)
            # temp_ans.append((nx, ny))
            vis_mp[nx][ny] = 1
            
            ans_mp[nx][ny] = ans_mp[now_pos[0][0]][now_pos[0][1]]
            ans_mp[nx][ny][index_mp[now_pos[0][0]][now_pos[0][1]]] = np.array((nx, ny))
            index_mp[nx][ny] = index_mp[now_pos[0][0]][now_pos[0][1]] + 1
            q = np.append(q, np.array([[np.array([nx, ny]), np.full(2, is_taking), now_pos[0]]]), axis=0)
    # log.write(f'{index_mp}\n')
    return ans_mp, index_mp

def check_one_line(point1, point2, point3):
    # all tuple
    # y2 - y1 / x2 - x1 = y3 - y1 / x3 - x1
    if (point2[1] - point1[1]) * (point3[0] - point1[0]) == (point3[1] - point1[1]) * (point2[0] - point1[0]):
        return 1
    return 0

def ignore_now_point(env_mp, point1, point2, point3, is_take_thing):
    # x_1 == x_3, y_1 == y_3
    min_x, max_x = min(point1[0], point3[0]), max(point1[0], point3[0])
    min_y, max_y = min(point1[1], point3[1]), max(point1[1], point3[1])
    if point1[0] == point3[0]:
        for i in range(min_y, max_y + 1):
            nx, ny = min_x, i
            if env_mp[nx][ny] == '#':
                return 0
            if is_take_thing == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and env_mp[nx + 1][ny] == '#'  and env_mp[nx - 1][ny] == '#')):
                return 0
    elif point1[1] == point3[1]:
        for i in range(min_x, max_x + 1):
            nx, ny = i, min_y
            if env_mp[nx][ny] == '#':
                return 0
            if is_take_thing == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and env_mp[nx][ny - 1] == '#'  and env_mp[nx][ny + 1] == '#')):
                return 0
    else:
        # y = kx +b
        # y_1 = kx_1 + b; y_2 = kx_2 +b; k = (y_2 - y_1) /(x_2 - x_1); b=y_2 - x_2*k
        k = (point3[1] - point1[1]) / (point3[0] - point1[0])
        b = point3[1] - point3[0] * k
        # log.write(f"{point1, point3}\n")
        for i in range(min_x, max_x):
            nx = i
            ny = int(k * nx + b)
            # log.write(f"{nx, ny}\n")
            # TODO: 可能会存在问题  # 确实有问题
            if env_mp[nx][ny] == '#':
                return 0
            if is_take_thing == 1 and ((ny + 1 >= cfg.MAP_SIZE and env_mp[nx][ny - 1] == '#') or (ny - 1 < 0 and env_mp[nx][ny + 1] == '#') or (ny + 1 < cfg.MAP_SIZE and ny - 1 >= 0 and env_mp[nx][ny - 1] == '#'  and env_mp[nx][ny + 1] == '#')):
                return 0
            if is_take_thing == 1 and ((nx + 1 >= cfg.MAP_SIZE and env_mp[nx - 1][ny] == '#') or (nx - 1 < 0 and env_mp[nx + 1][ny] == '#') or (nx + 1 < cfg.MAP_SIZE and nx - 1 >= 0 and env_mp[nx + 1][ny] == '#'  and env_mp[nx - 1][ny] == '#')):
                return 0
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
                pre_point = now_point
                if pos + 1 >= path_len:
                    break
                else:
                    shr_point = nxt_point = path_list[pos + 1]
    new_path.append(nxt_point)
    # log.write(f"{new_path}\n")
    return new_path

def path_better_np(env_mp, path_list, is_take_thing):
    if len(path_list) <= 1:
        return path_list
    # log.write(f"{path_list}\n")
    path_len = len(path_list)
    new_path = np.full((cfg.MAX_ANS_MP_SIZE, 2), -1, dtype = int)
    pre_point = path_list[0]
    new_path[0] = pre_point
    shr_point = path_list[1]
    nxt_point = shr_point

    counter = 1
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
                new_path[counter] = nxt_point
                counter += 1
                pre_point = now_point
                if pos + 1 >= path_len:
                    break
                else:
                    shr_point = nxt_point = path_list[pos + 1]
    new_path[counter] = nxt_point
    counter += 1
    # log.write(f"{new_path}\n")
    return new_path, counter
            
            