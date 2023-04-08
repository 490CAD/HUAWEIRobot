import math
import sys
from robot import Robot
from workbench import WorkBench
from config import CFG
from queue import Queue
import numpy as np
from collections import deque
import heapq
import time
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
    # eps = 1e-6 if mode != 2 else 1e-4
    eps = 1e-4
    dy = target_y - origin_y
    dx = target_x - origin_x
    ans = math.atan2(dy, dx)
    if abs(abs(ans) - cfg.PI) <= eps:
        ans = cfg.PI
    # if abs(ans + cfg.PI) < eps:
    #     return -ans
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

def astar(env_mp, st_pos, ed_pos, is_take_thing, mask_env_mp, map_limit):
    log.write("ASTART!\n")
    d1 = time.time()
    q, ans_mp, vis_mp, open_list, close_list = [], {}, {}, {}, set()
    heapq.heappush(q, (0, 0, st_pos))
    open_list[st_pos] = 0
    vis_mp[st_pos] = 1
    ans_path = []
    while len(q) != 0:
        now_pos = heapq.heappop(q)
        x, y = now_pos[2][0], now_pos[2][1]
        log.write(f"{x, y}\n")
        fvalue = open_list[(x, y)]
        if x == ed_pos[0] and y == ed_pos[1]:
            # 返回路径这里
            while (x, y) in ans_mp:
                ans_path.append((x, y))
                x, y = ans_mp[(x, y)][0], ans_mp[(x, y)][1]
            ans_path.append(st_pos)
            ans_path.reverse()
            d2 = time.time()
            log.write(f"{'{:.10f}s.'.format(d2 - d1)}\n")
            log.write("AENDED!\n")
            return ans_path
        if (x, y) in close_list:
            continue
        close_list.add((x, y))
        for i in range(8):
            nx, ny = x + cfg.DIS_HIGHER[i][0], y + cfg.DIS_HIGHER[i][1]
            if nx < map_limit[0] or ny < map_limit[2] or nx > map_limit[1] or ny > map_limit[3] or (nx, ny) in close_list:
                continue
            if is_take_thing == 1 and mask_env_mp[nx][ny] == 0:
                continue
            if is_take_thing == 0 and check_points(env_mp, nx, ny, is_take_thing) == 0:
                continue
            gi = cfg.gvalue[i]
            hi = abs(nx - ed_pos[0]) + abs(ny - ed_pos[1])
            qi = 0 #周围墙的数量
            if (nx, ny) not in open_list or fvalue < gi + hi + qi:
                open_list[(nx, ny)] = gi + hi + qi
                vis_mp[(nx, ny)] = vis_mp[(x, y)] + 1
                ans_mp[(nx, ny)] = now_pos[2]
                heapq.heappush(q, (gi + hi + qi, qi, (nx, ny)))
    return ans_path
    


def bfs(env_mp, st_pos, is_take_thing, mask_env_mp, map_limit):
    q = deque()
    q.append(st_pos)
    # log.write(f"{len(env_mp), len(env_mp[0])}\n")
    ans_mp = [[None for i in range(cfg.MAP_SIZE_2)] for j in range(cfg.MAP_SIZE_2)]
    vis_mp = [[0 for i in range(cfg.MAP_SIZE_2)] for j in range(cfg.MAP_SIZE_2)]
    vis_mp[st_pos[0]][st_pos[1]] = 1
    ans_mp[st_pos[0]][st_pos[1]] = st_pos
    while len(q) != 0:
        now_pos = q.popleft()
        x, y = now_pos[0], now_pos[1]
        for i in range(8):
            nx, ny = x + cfg.DIS_HIGHER[i][0], y + cfg.DIS_HIGHER[i][1]
            # nx, ny = now_pos[0][0] + cfg.DIS_NORMAL[i][0], now_pos[0][1] + cfg.DIS_NORMAL[i][1]
            if nx < map_limit[0] or ny < map_limit[2] or nx > map_limit[1] or ny > map_limit[3] or vis_mp[nx][ny] != 0:
                continue
            if is_take_thing == 1 and mask_env_mp[nx][ny] == 0:
                continue
            if is_take_thing == 0 and check_points(env_mp, nx, ny, is_take_thing) == 0:
                continue
            vis_mp[nx][ny] = vis_mp[x][y] + 1
            ans_mp[nx][ny] = now_pos
            q.append((nx, ny))     
    return ans_mp, vis_mp
            
            
def ask_path(ed_pos, ans_mp, env_mp, mask_env_mp):
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
    # return path
    path = path_better(env_mp, path, 1, mask_env_mp)
    log.write(f"{path}\n")
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
    nx3, nx4 = max(0, nx - 2), min(cfg.MAP_SIZE_2 - 1, nx + 2)
    ny3, ny4 = max(0, ny - 2), min(cfg.MAP_SIZE_2 - 1, ny + 2)
    if is_take_thing == 1:
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
        if env_mp[nx3][ny3] == '#' or env_mp[nx3][ny4] == '#' or env_mp[nx4][ny3] == '#' or env_mp[nx4][ny4] == '#':
            return 0
    return 1

def ignore_now_point(env_mp, point1, point2, point3, is_take_thing, mask_env_mp):
    # x_1 == x_3, y_1 == y_3
    min_x, max_x = min(point1[0], point3[0]), max(point1[0], point3[0])
    min_y, max_y = min(point1[1], point3[1]), max(point1[1], point3[1])
    if point1[0] == point3[0]:
        for i in range(min_y, max_y + 1):
            nx, ny = min_x, i
            if is_take_thing == 1 and mask_env_mp[nx][ny]:
                return 0
            if is_take_thing == 0 and check_points(env_mp, nx, ny, is_take_thing) == 0:
                return 0
    elif point1[1] == point3[1]:
        for i in range(min_x, max_x + 1):
            nx, ny = i, min_y
            if is_take_thing == 1 and mask_env_mp[nx][ny]:
                return 0
            if is_take_thing == 0 and check_points(env_mp, nx, ny, is_take_thing) == 0:
                return 0
    else:
        # return 0
        # y = kx +b
        # y_1 = kx_1 + b; y_2 = kx_2 +b; k = (y_2 - y_1) /(x_2 - x_1); b=y_2 - x_2*k
        k = (point3[1] - point1[1]) / (point3[0] - point1[0])
        b1 = point3[1] - point3[0] * k
        b2 = (2.4) * math.sqrt(k * k + 1) + b1
        b3 = -(2.4) * math.sqrt(k * k + 1) + b1
        # b2 = point3[1] - point3[0] * k + 0.53
        # b3 = point3[1] - point3[0] * k - 0.53
        # log.write(f"{point1, point3}\n")
        for i in range(min_x, max_x):
            nx = i
            ny1 = (k * nx + b1)
            ny2 = (k * nx + b2)
            ny3 = (k * nx + b3)
            int_ny = int(ny1)

            # if int_ny == ny1:
            #     ny = int_ny
            #     if is_take_thing == 1 and mask_env_mp[nx][ny] == 0:
            #         return 0
            #     if is_take_thing == 0 and check_points(env_mp, nx, ny, is_take_thing) == 0:
            #         return 0
            # else:
            ny = max(0, int_ny)
            ny = min(ny, cfg.MAP_SIZE_2 - 1)
            
            if is_take_thing == 1 and (mask_env_mp[nx][ny] == 0 or mask_env_mp[nx][min(ny + 1, cfg.MAP_SIZE_2 - 1)] == 0 or mask_env_mp[nx][max(0, ny - 1)] == 0):
                return 0
            if is_take_thing == 0 and (check_points(env_mp, nx, ny, is_take_thing) == 0 or check_points(env_mp, nx, min(ny + 1, cfg.MAP_SIZE_2 - 1), is_take_thing) == 0 or check_points(env_mp, nx, max(0, ny - 1), is_take_thing) == 0):
                return 0
            int_ny = int(ny2)
            ny = max(0, int_ny)
            ny = min(ny, cfg.MAP_SIZE_2 - 1)
            
            if is_take_thing == 1 and (mask_env_mp[nx][ny] == 0 or mask_env_mp[nx][min(ny + 1, cfg.MAP_SIZE_2 - 1)] == 0 or mask_env_mp[nx][max(0, ny - 1)] == 0):
                return 0
            if is_take_thing == 0 and (check_points(env_mp, nx, ny, is_take_thing) == 0 or check_points(env_mp, nx, min(ny + 1, cfg.MAP_SIZE_2 - 1), is_take_thing) == 0 or check_points(env_mp, nx, max(0, ny - 1), is_take_thing) == 0):
                return 0
            int_ny = int(ny3)
            ny = max(0, int_ny)
            ny = min(ny, cfg.MAP_SIZE_2 - 1)
            
            if is_take_thing == 1 and (mask_env_mp[nx][ny] == 0 or mask_env_mp[nx][min(ny + 1, cfg.MAP_SIZE_2 - 1)] == 0 or mask_env_mp[nx][max(0, ny - 1)] == 0):
                return 0
            if is_take_thing == 0 and (check_points(env_mp, nx, ny, is_take_thing) == 0 or check_points(env_mp, nx, min(ny + 1, cfg.MAP_SIZE_2 - 1), is_take_thing) == 0 or check_points(env_mp, nx, max(0, ny - 1), is_take_thing) == 0):
                return 0
            
        # log.write("-------------------\n")
    
    return 1

def path_better(env_mp, path_list, is_take_thing, mask_env_mp):
    log.write(f"{path_list}\n")
    path_len = len(path_list)
    new_path = []
    if path_len == 0:
        return path_list
    if path_len == 1:
        log.write(f"path_len==1{path_list}\n")
        path_list[0] = (cal_x(path_list[0][1] / 2), cal_y(path_list[0][0] / 2))
        return path_list
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
            if ignore_now_point(env_mp, pre_point, shr_point, now_point, is_take_thing, mask_env_mp) == 1:
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
    for i in range(len(new_path)):
        new_path[i] = (cal_x(new_path[i][1] / 2), cal_y(new_path[i][0] / 2))

    # log.write(f"{new_path}\n")
    return new_path

def path_better_sxw(path_list, is_take_thing, walls):
    path_len = len(path_list)
    if path_len <= 1:
        return path_list
    
    # log.write(f"{path_list}\n")
    # temp_path, new_path, added_path = OrderedDict(), OrderedDict(), []
    temp_path, new_path, added_path, vis_path = {}, {}, [], {}
    val_path = {}
    temp_path[0], temp_path[path_len - 1] = path_list[0], path_list[path_len - 1]
    for i in range(path_len):
        vis_path[i] = -1
    vis_path[0], vis_path[path_len - 1] = 0, 0
    val_path[0], val_path[path_len - 1] = 5, 1
    cnt = 0
    while True:
        # cnt += 1
        # if cnt >= 500:
        #     break
        pre_point, pre_key, aft_point, aft_key = None, 0, None, 0
        temp_path_list = sorted(temp_path.items())

        # for key in temp_path_list:
        #     log.write(f"{key[0]} ")
        # log.write(f"\n")

        for key, point in temp_path_list:
            if pre_point is None:
                pre_point, pre_key = point, key
            elif aft_point is None:
                aft_point, aft_key = point, key
            else:
                if (val_path[pre_key] == 1 or val_path[pre_key] == 6) and (val_path[aft_key] == 5 or val_path[aft_key] == 6):
                    pass
                elif is_line_valid(walls, (cal_x(pre_point[1]), cal_y(pre_point[0])), (cal_x(aft_point[1]), cal_y(aft_point[0])), is_take_thing=is_take_thing) == True:
                    new_path[pre_key] = pre_point
                    val_path[pre_key] += 1
                    val_path[aft_key] += 5
                else:
                    kk = int((pre_key + aft_key) / 2)
                    if vis_path[kk] == -1:
                        vis_path[kk] = 0
                        val_path[kk] = 0
                        added_path.append(kk)
                # log.write(f"{pre_key, val_path[pre_key], aft_key, val_path[aft_key]}\n")
                pre_point, pre_key = aft_point, aft_key
                aft_point, aft_key = point, key
        # log.write(f"\n")

        if (val_path[pre_key] == 1 or val_path[pre_key] == 6) and (val_path[aft_key] == 5 or val_path[aft_key] == 6):
            pass
        elif is_line_valid(walls, (cal_x(pre_point[1]), cal_y(pre_point[0])), (cal_x(aft_point[1]), cal_y(aft_point[0])), is_take_thing=is_take_thing) == True:
            new_path[pre_key], new_path[aft_key] = pre_point, aft_point
            val_path[pre_key] += 1
            val_path[aft_key] += 5
        else:
            kk = int((pre_key + aft_key) / 2)
            if vis_path[kk] == -1:
                vis_path[kk] = 0
                val_path[kk] = 0
                added_path.append(kk)
        # log.write(f"{new_path}\n")
        if len(added_path) != 0:
            for key in added_path:
                temp_path[key] = path_list[key]
            added_path.clear()
        # temp_path = sorted(temp_path.items())
        flag = 0
        for key in val_path.keys():
            # log.write(f"{key}:{ val_path[key]}; ")
            if val_path[key] != 6:
                flag = 1
                break
        # log.write(f"\n")
        if flag == 0:
            break
        log.write(f"val_path is {val_path}\n")
        log.write(f"temp is {temp_path_list}\n")
    new_path_list = sorted(new_path.items())
    _path = []
    for point in new_path_list:
        _path.append(point[1])
    log.write(f"new_path_list is {_path}\n")

    return _path

def ask_path_sxw(ed_pos, ans_mp, is_take_thing, walls):
    path = []
    path.append(ed_pos)
    pos = ed_pos
    nx, ny = pos[0], pos[1]
    while ans_mp[(nx, ny)] != pos:
        pos = ans_mp[(nx, ny)]
        nx, ny = pos[0], pos[1]
        path.append(pos)
    path.reverse()
    # return path
    log.write(f"origin path is {path}\n")
    _len = len(path)
    if _len > 2:
        for i in range(1):
            path = path_better_sxw(path, is_take_thing, walls)
    len_ = len(path)
    path_ = []
    if len_ > 2:
        pre, now, aft = path[0], path[1], path[2]
        cnt = 2
        while cnt < len_:
            if check_one_line(pre, now, aft) == 1:
                if cnt == len_ - 1:
                    path_.append(pre)
                    path_.append(now)
                    path_.append(aft)
                    cnt += 1
                else:
                    now, aft = aft, path[cnt + 1]
                    cnt += 1
            else:
                if cnt == len_ - 1:
                    path_.append(pre)
                    path_.append(now)
                    path_.append(aft)
                    cnt += 1
                else:
                    path_.append(pre)
                    pre, now, aft = now, aft, path[cnt + 1]
                    cnt += 1
    else:
        path_ = path
                    
                
    log.write(f"better path is {path_}\n")

    for i in range(len(path_)):
        path_[i] = (cal_x(path_[i][1]), cal_y(path_[i][0]))

    return path_

def find_block(point0:list, point1:list, d=0.45+np.sqrt(2)*0.25) -> list:
    """
    找到一条线段附近的区域
    d: 平移的距离
    """
    x0, y0 = point0[0], point0[1]
    x1, y1 = point1[0], point1[1]
    if x0 == x1:
        x_min, x_max = x0 - d, x0 + d
        if y0 < y1:
            y_min, y_max = y0, y1
        else:
            y_min, y_max = y1, y0
    elif y0 == y1:
        y_min, y_max = y0 - d, y0 + d
        if x0 < x1:
            x_min, x_max = x0, x1
        else:
            x_min, x_max = x1, x0
    else:
        k = (y1 - y0) / (x1 - x0)
        b = (x1 * y0 - x0 * y1)/(x1 - x0)
        line1_1_x = (math.sqrt(k * k + 1) * d + y0 + x0 / k - b) / (k + 1 / k)
        line1_1_y = (x0 - line1_1_x) / k + y0
        line1_2_x = (y0 + x0 / k - b - math.sqrt(k * k + 1) * d) / (k + 1 / k)
        line1_2_y = (x0 - line1_2_x) / k + y0
        line2_1_x = (math.sqrt(k * k + 1) * d + y1 + x1 / k - b) / (k + 1 / k)
        line2_1_y = (x1 - line2_1_x) / k + y1
        line2_2_x = (y1 + x1/ k - b - math.sqrt(k * k + 1) * d) / (k + 1 / k)
        line2_2_y = (x1 - line2_2_x) / k + y1
        x_collection = [line1_1_x, line1_2_x, line2_1_x, line2_2_x]
        y_collection = [line1_1_y, line1_2_y, line2_1_y, line2_2_y]
        x_min = min(x_collection)
        x_max = max(x_collection)
        y_min = min(y_collection)
        y_max = max(y_collection)
    return [x_min, x_max, y_min, y_max]

def is_line_crash_wall(wall:object, point0:tuple, point1:tuple, angle:float, tolerance=0.45)->bool:
    log.write(f"angle is {angle}\n")
    x0, y0 = point0[0], point0[1]
    x1, y1 = point1[0], point1[1]
    dis_point = 0
    if x0 == x1:
        dis_point = abs(x0 - wall.x)
    elif y0 == y1:
        dis_point = abs(y0 - wall.y)
    else:
        k = (y1 - y0) / (x1 - x0)
        b = (x1 * y0 - x0 * y1)/(x1 - x0)
        dis_point = abs((-k*wall.x + wall.y - b)/math.sqrt(k**2 + 1))
    if angle >=0 and angle < cfg.PI / 2:
        dis_final = dis_point - math.sqrt(2) * 0.25 * math.cos(abs(cfg.PI / 4 - angle))
    elif angle >= cfg.PI / 2 and angle <= cfg.PI:
        dis_final = dis_point - math.sqrt(2) * 0.25 * math.cos(abs(cfg.PI / 4 * 3 - angle))
    log.write(f"dis_final is {dis_final}\n")
    if dis_final > tolerance:
        return True
    else:
        return False
    

def is_line_valid(walls:list, point0:tuple, point1:tuple, d=0.45+math.sqrt(2)*0.25, is_take_thing = 0)->bool:
    if point0 == point1 or (abs(point0[0] - point1[0])<=0.25 and abs(point0[1] - point1[1])<=0.25):
        return True
    log.write(f"is_take_thing is {is_take_thing}\n")
    x0, y0 = point0[0], point0[1]
    x1, y1 = point1[0], point1[1]
    tolerance = 0.45
    if is_take_thing == 1:
        d = 0.53 + math.sqrt(2) * 0.25
        tolerance = 0.53
    y_dis = y1 - y0
    x_dis = x1 - x0
    if y_dis == -0.0:
        y_dis = 0.0
    angle = math.atan2(y_dis, x_dis)
    if angle < 0:
        angle = angle + cfg.PI
    [x_min, x_max, y_min, y_max] = find_block(point0, point1, d=d)
    log.write(f"x_min, x_max, y_min, y_max is {x_min}, {x_max}, {y_min}, {y_max}\n")
    for wall in walls:
        if wall.x > x_min and wall.x < x_max and wall.y > y_min and wall.y < y_max:
            log.write(f"wall_id is {wall.wall_id}\n")
            if is_line_crash_wall(wall, point0, point1, angle, tolerance=tolerance) == False:
                # log.write(f"return false, point0, point1 is {point0}, {point1}\n")
                return False
    return True    
    

def check_points_half(env_mp, nx, ny, is_take_thing, debug_mode=0):
    # log.write('-\n')
    # log.write(f"{len(env_mp[0]), len(env_mp)}\n")
    int_nx, int_ny = int(nx), int(ny)
    if int(nx * 10) == int_nx * 10 and int(ny * 10) == int_ny * 10:
        # log.write(f"1: {nx, int_nx}, {ny, int_ny}\n")
        nx, ny = int_nx, int_ny
        nx1, nx2, nx3, nx4 = min(cfg.MAP_SIZE - 1, int_nx + 1), max(0, int_nx - 1), min(cfg.MAP_SIZE - 1, int_nx + 2), max(0, int_nx - 2)
        ny1, ny2, ny3, ny4 = min(cfg.MAP_SIZE - 1, int_ny + 1), max(0, int_ny - 1), min(cfg.MAP_SIZE - 1, int_ny + 2), max(0, int_ny - 2)

        if int_nx == 37 and int_ny == 22 and debug_mode == 1:
            log.write('~\n')
            log.write(f"{nx1, nx2, nx3, nx4}\n")
            log.write(f"{ny1, ny, ny2}\n")
            for i in range(ny2, ny1 + 1):
                log.write(f"{env_mp[nx4][i]}")
            log.write("\n")
            for i in range(ny2, ny1 + 1):
                log.write(f"{env_mp[nx2][i]}")
            log.write("\n")
            for i in range(ny2, ny1 + 1):
                log.write(f"{env_mp[nx1][i]}")
            log.write("\n")
            for i in range(ny2, ny1 + 1):
                log.write(f"{env_mp[nx3][i]}")
            log.write("\n")
            log.write('~\n')
        if env_mp[nx][ny1] == '#' or env_mp[nx][ny2] == '#' or env_mp[nx1][ny] == '#' or env_mp[nx2][ny] == '#':
            return 0
        if env_mp[nx1][ny1] == '#' or env_mp[nx1][ny2] == '#' or env_mp[nx2][ny1] == '#' or env_mp[nx2][ny2] == '#':
            return 0
        # if is_take_thing == 1 and (env_mp[nx][ny3] == '#' or env_mp[nx][ny4] == '#' or env_mp[nx3][ny] == '#' or env_mp[nx4][ny] == '#'):
        #     return 0
        # if is_take_thing == 1 and (env_mp[nx1][ny3] == '#' or env_mp[nx2][ny4] == '#' or env_mp[nx3][ny1] == '#' or env_mp[nx4][ny2] == '#'):
        #     return 0
    elif int(nx * 10) == int_nx * 10:

        if debug_mode == 1 and nx == 37.0 and ny == 22.0:
            log.write(f"sb\n")
        # log.write(f"2: {nx, int_nx}, {ny, int_ny}\n")
        nx = int_nx
        int_ny = int(ny - 0.5)
        nx1, nx2 = min(cfg.MAP_SIZE - 1, int_nx + 1), max(0, int_nx - 1)
        ny1, ny2, ny3, ny4 = min(cfg.MAP_SIZE - 1, int_ny + 1), max(0, int_ny), min(cfg.MAP_SIZE - 1, int_ny + 2), max(0, int_ny - 1)
        # log.write(f"{nx1, nx2}\n")
        # log.write(f"{nx, ny, ny1, ny2, ny3, ny4}\n")
        if env_mp[nx][ny2] == '#' or env_mp[nx][ny1] == '#' or env_mp[nx1][ny2] == '#' or env_mp[nx1][ny1] == '#' or env_mp[nx2][ny2] == '#' or env_mp[nx2][ny1] == '#':
            return 0
        if is_take_thing == 1 and (env_mp[nx][ny3] == '#' or env_mp[nx][ny4] == '#'):
            return 0
    elif int(ny * 10) == int_ny * 10:
        # log.write(f"3: {nx, int_nx}, {ny, int_ny}\n")

        
        int_nx = int(nx - 0.5)
        ny = int_ny
        nx1, nx2, nx3, nx4 = min(cfg.MAP_SIZE - 1, int_nx + 1), max(0, int_nx), min(cfg.MAP_SIZE - 1, int_nx + 2), max(0, int_nx - 1)
        ny1, ny2 = min(cfg.MAP_SIZE - 1, int_ny + 1), max(0, int_ny - 1)
        if env_mp[nx1][ny] == '#' or env_mp[nx2][ny] == '#' or env_mp[nx1][ny1] == '#' or env_mp[nx2][ny1] == '#' or env_mp[nx1][ny2] == '#' or env_mp[nx2][ny2] == '#':
            if nx == 36.5 and ny == 22 and debug_mode == 1:
                log.write('~\n')
                log.write(f"{nx1, nx2, nx3, nx4}\n")
                log.write(f"{ny1, ny, ny2}\n")
                for i in range(ny2, ny1 + 1):
                    log.write(f"{env_mp[nx4][i]}")
                log.write("\n")
                for i in range(ny2, ny1 + 1):
                    log.write(f"{env_mp[nx2][i]}")
                log.write("\n")
                for i in range(ny2, ny1 + 1):
                    log.write(f"{env_mp[nx1][i]}")
                log.write("\n")
                for i in range(ny2, ny1 + 1):
                    log.write(f"{env_mp[nx3][i]}")
                log.write("\n")
                log.write('~\n')
            return 0
        if is_take_thing == 1 and (env_mp[nx3][ny] == '#' or env_mp[nx4][ny] == '#'):
            return 0
    else:
        # log.write(f"4: {nx, int_nx}, {ny, int_ny}\n")
        int_nx, int_ny = int(nx - 0.5), int(ny - 0.5)
        nx1, nx2, nx3, nx4 = min(cfg.MAP_SIZE - 1, int_nx + 1), max(0, int_nx), min(cfg.MAP_SIZE - 1, int_nx + 2), max(0, int_nx - 1)
        ny1, ny2, ny3, ny4 = min(cfg.MAP_SIZE - 1, int_ny + 1), max(0, int_ny), min(cfg.MAP_SIZE - 1, int_ny + 2), max(0, int_ny - 1)
        if env_mp[nx2][ny2] == '#' or env_mp[nx2][ny1] == '#' or env_mp[nx1][ny2] == '#' or env_mp[nx1][ny1] == '#':
            return 0
        if is_take_thing == 1 and (env_mp[nx2][ny3] == '#' or env_mp[nx2][ny4] == '#' or env_mp[nx1][ny3] == '#' or env_mp[nx1][ny4] == '#' or env_mp[nx3][ny2] == '#' or env_mp[nx3][ny1] == '#' or env_mp[nx4][ny1] == '#' or env_mp[nx4][ny2] == '#'):
            return 0

    return 1

def bfs_half(env_mp, st_pos, is_take_thing, map_limit, debug_mode=0):
    if debug_mode==1:
        log.write("----\n")
    q = deque()
    q.append(st_pos)
    ans_mp, vis_mp = {}, {}
    vis_mp[st_pos] = 1
    ans_mp[st_pos] = st_pos
    while len(q) != 0:
        now_pos = q.popleft()
        x, y = now_pos[0], now_pos[1]
        cnt = 0
        for i in range(8):
            nx, ny = x + cfg.DIS_HALF[i][0], y + cfg.DIS_HALF[i][1]
            # nx, ny = now_pos[0][0] + cfg.DIS_NORMAL[i][0], now_pos[0][1] + cfg.DIS_NORMAL[i][1]
            if x == 36.5 and y == 22 and debug_mode == 1:
                log.write(f"{nx, ny, cnt, vis_mp.get((nx, ny))}.\n")
            if nx < map_limit[0] or ny < map_limit[2] or nx >= map_limit[1] or ny >= map_limit[3] or vis_mp.get((nx, ny)) is not None:
                continue
            if check_points_half(env_mp, nx, ny, is_take_thing, debug_mode) == 0:
                continue
            if x == 36.5 and y == 22 and debug_mode==1:
                log.write(f"{nx, ny, cnt}!!\n")
            cnt += 1
            vis_mp[(nx, ny)]= vis_mp[(x, y)] + 1
            ans_mp[(nx, ny)]= now_pos
            q.append((nx, ny))     
    
    if debug_mode==1:
        log.write("----\n")
    return ans_mp, vis_mp


def ask_path_half(ed_pos, ans_mp, env_mp, is_take_thing):
    path = []
    path.append(ed_pos)
    pos = ed_pos
    nx, ny = pos[0], pos[1]
    while ans_mp[(nx, ny)] != pos:
        pos = ans_mp[(nx, ny)]
        nx, ny = pos[0], pos[1]
        path.append(pos)
    path.reverse()
    path = path_better_half(env_mp, path, is_take_thing)

    # for i in range(len(path)):
    #     path[i] = (cal_x(path[i][1]), cal_y(path[i][0]))

    return path

def path_better_half(env_mp, path_list, is_take_thing):
    # log.write(f"{path_list}\n")
    path_len = len(path_list)
    new_path = []
    if path_len == 0:
        return path_list
    if path_len == 1:
        # log.write(f"path_len==1{path_list}\n")
        path_list[0] = (cal_x(path_list[0][1]), cal_y(path_list[0][0]))
        return path_list
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
            if ignore_now_point_half(env_mp, pre_point, now_point, is_take_thing) == 1:
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
    for i in range(len(new_path)):
        new_path[i] = (cal_x(new_path[i][1]), cal_y(new_path[i][0]))

    # log.write(f"{new_path}\n")
    return new_path


def ignore_now_point_half(env_mp, point1, point3, is_take_thing):
    # x_1 == x_3, y_1 == y_3
    min_x, max_x = min(point1[0], point3[0]), max(point1[0], point3[0])
    min_y, max_y = min(point1[1], point3[1]), max(point1[1], point3[1])
    if point1[0] == point3[0]:
        for i in range(int(min_x),int(max_x)):
            nx, ny = min_x, i
            if check_points_half(env_mp, nx, ny, is_take_thing) == 0:
                return 0
    elif point1[1] == point3[1]:
        for i in range(int(min_x),int(max_x)):
            nx, ny = i, min_y
            if check_points_half(env_mp, nx, ny, is_take_thing) == 0:
                return 0
    else:
        k = (point3[1] - point1[1]) / (point3[0] - point1[0])
        b1 = point3[1] - point3[0] * k
        b2 = (1) * math.sqrt(k * k + 1) + b1
        b3 = -(1) * math.sqrt(k * k + 1) + b1
        for i in range(int(min_x),int(max_x)):
            nx = i
            ny1 = (k * nx + b1)
            ny2 = (k * nx + b2)
            ny3 = (k * nx + b3)
            int_ny = int(ny1)
            ny = max(0, int_ny)
            ny = min(ny, cfg.MAP_SIZE - 1)

            if check_points_half(env_mp, nx, ny, is_take_thing) == 0 or check_points_half(env_mp, nx, min(ny + 1, cfg.MAP_SIZE - 1), is_take_thing) == 0 or check_points_half(env_mp, nx, max(0, ny - 1), is_take_thing) == 0:
                return 0
            int_ny = int(ny2)
            ny = max(0, int_ny)
            ny = min(ny, cfg.MAP_SIZE - 1)

            if check_points_half(env_mp, nx, ny, is_take_thing) == 0 or check_points_half(env_mp, nx, min(ny + 1, cfg.MAP_SIZE - 1), is_take_thing) == 0 or check_points_half(env_mp, nx, max(0, ny - 1), is_take_thing) == 0:
                return 0
            int_ny = int(ny3)
            ny = max(0, int_ny)
            ny = min(ny, cfg.MAP_SIZE - 1)

            if check_points_half(env_mp, nx, ny, is_take_thing) == 0 or check_points_half(env_mp, nx, min(ny + 1, cfg.MAP_SIZE - 1), is_take_thing) == 0 or check_points_half(env_mp, nx, max(0, ny - 1), is_take_thing) == 0:
                return 0
            
            
    
    return 1


def astar_half(env_mp, st_pos, ed_pos, is_take_thing, map_limit):
    log.write("ASTART!\n")
    d1 = time.time()
    q, ans_mp, vis_mp, open_list, close_list = [], {}, {}, {}, set()
    heapq.heappush(q, (0, 0, st_pos))
    open_list[st_pos] = 0
    vis_mp[st_pos] = 1
    ans_path = []
    while len(q) != 0:
        now_pos = heapq.heappop(q)
        x, y = now_pos[2][0], now_pos[2][1]
        log.write(f"{x, y}\n")
        fvalue = open_list[(x, y)]
        if x == ed_pos[0] and y == ed_pos[1]:
            # 返回路径这里
            while (x, y) in ans_mp:
                ans_path.append((x, y))
                x, y = ans_mp[(x, y)][0], ans_mp[(x, y)][1]
            ans_path.append(st_pos)
            ans_path.reverse()
            d2 = time.time()
            log.write(f"{'{:.10f}s.'.format(d2 - d1)}\n")
            log.write("AENDED!\n")
            return ans_path
        if (x, y) in close_list:
            continue
        close_list.add((x, y))
        for i in range(8):
            nx, ny = x + cfg.DIS_HIGHER[i][0], y + cfg.DIS_HIGHER[i][1]
            if nx < map_limit[0] or ny < map_limit[2] or nx > map_limit[1] or ny > map_limit[3] or (nx, ny) in close_list:
                continue
            if check_points_half(env_mp, nx, ny, is_take_thing) == 0:
                continue
            gi = cfg.gvalue[i]
            hi = abs(nx - ed_pos[0]) + abs(ny - ed_pos[1])
            qi = 0 #周围墙的数量
            if (nx, ny) not in open_list or fvalue > gi + hi + qi:
                open_list[(nx, ny)] = gi + hi + qi
                vis_mp[(nx, ny)] = vis_mp[(x, y)] + 1
                ans_mp[(nx, ny)] = now_pos[2]
                heapq.heappush(q, (gi + hi + qi, qi, (nx, ny)))
    return ans_path