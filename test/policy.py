# oldest version policy
def get_simple_job(free_robots):
    robot_id, target0, target1 = -1, -1, -1

    job_list = dict(sorted(simple_job_workbenchs.items(),key=lambda x:x[1],reverse=True))
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
            target1 = find_nearest_target_sell(workbenchs[target0].x ,workbenchs[target0].y, target_workbench_list, workbenchs[target0].work_type)
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

def get_job(free_robots, free_jobs):
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
        target1 = find_nearest_target_sell(workbenchs[target0].x ,workbenchs[target0].y, target_workbench_list, workbenchs[target0].work_type)
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

 
# distance policy
def get_price_simple_by_dis(free_robots):
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
            target1 = find_nearest_target_sell(workbenchs[target0].x ,workbenchs[target0].y, target_workbench_list, workbenchs[target0].work_type)
            
            if target1 == -1:
                continue
            all_dis = robot_dis + DIS_MP[target0][target1]
            temp_val = cfg.THING_VALUE[workbenchs[target0].work_type]
            temp_val_dis = temp_val / all_dis
            if temp_val_dis > best_val_dis:
                robot_id, target0_id, target1_id = robot, target0, target1
                best_val_dis = temp_val_dis

    return robot_id, target0_id, target1_id

def get_price_by_dis(free_robots):
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
            target1 = find_nearest_target_sell(workbenchs[workbench].x ,workbenchs[workbench].y, target_workbench_list, workbenchs[workbench].work_type)
            if target1 == -1:
                continue
            all_dis = robot_dis + DIS_MP[target0][target1]
            temp_val = cfg.THING_VALUE[workbenchs[workbench].work_type]
            temp_val_dis = temp_val / all_dis
            if temp_val_dis > best_val_dis:
                robot_id, target0_id, target1_id = robot, target0, target1
                best_val_dis = temp_val_dis
    return robot_id, target0_id, target1_id

# by time
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




def get_price_by_look_further(free_robots):
    robot_id, target0_id, target1_id, best_val_time = -1, -1, -1, 0.0
    workbench_list = useful_workbench_list
    target2_id = -1
    for id in free_robots:
        log.write(f"**robot {id}** \n")
        log.write(f"workbench_list: {workbench_list}\n")
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
                log.write(f"target0: {target0}; target1: {target1}\n")
                target0_target1_dis = DIS_MP[target0][target1]
                robot_target0_dis = cal_point_x_y(robot.x, robot.y, target0_workbench.x, target0_workbench.y)
     
                robot_target0_time = robot_target0_dis * 50 / 6
                wait_target0_time = max(target0_workbench.remain_time - robot_target0_time, 0)

                target0_val = cfg.THING_VALUE[target0_workbench.work_type]
                go_from_target0_time = target0_target1_dis * 50 / 6
                
                temp_time = robot_target0_time + wait_target0_time + go_from_target0_time
                temp_val_time = target0_val / temp_time

                flag = 0
                if temp_val_time > best_val_time:
                    if target0_workbench.work_type in cfg.HIGH_LEVEL_WORKBENCH:
                        if target0_workbench.work_type == 7:
                            target2_workbench_list = [4, 5, 6]
                        elif target0_workbench.work_type == 4:
                            target2_workbench_list = [1, 2]
                        elif target0_workbench.work_type == 5:
                            target2_workbench_list = [1, 3]
                        elif target0_workbench.work_type == 6:
                            target2_workbench_list = [2, 3]
                        target2_ava_list = get_ava_list(target2_workbench_list, workbench_type_num)
                        for target2 in target2_ava_list:
                            target2_workbench = workbenchs[target2]
                            if target2_workbench.is_targeted_flag[0] == 1 or target0_workbench.is_targeted_flag[target2_workbench.work_type] == 1:
                                continue
                            if target2_workbench.output != 1 and target2_workbench.remain_time == -1:
                                continue
                            if ((1 << target2_workbench.work_type) & target0_workbench.origin_thing) != 0:
                                continue
                            target2_target0_dis = DIS_MP[target2][target0]
                            robot_target2_dis = cal_point_x_y(robot.x, robot.y, target2_workbench.x, target2_workbench.y)
                            go_from_robot_time = robot_target2_dis * 50 / 6
                            go_from_target2_time = target2_target0_dis * 50 / 6
                            wait_target2_time = max(target2_workbench.remain_time - go_from_robot_time, 0)
                            wait_t2_target0_time = max(target0_workbench.remain_time - wait_target2_time - go_from_robot_time - go_from_target2_time, 0)
                            target2_val = cfg.THING_VALUE[target2_workbench.work_type]

                            ttemp_time = go_from_robot_time + wait_target2_time + go_from_target0_time + wait_t2_target0_time + go_from_target0_time
                            ttemp_val_time = (target2_val + target0_val) / ttemp_time

                            if ttemp_val_time > temp_val_time:
                                robot_id, target0_id, target1_id, target2_id = id, target0, target1, target2
                                best_val_time = ttemp_val_time
                                flag = 1
                    if flag == 0:
                        robot_id, target0_id, target1_id = id, target0, target1
                        best_val_time = temp_val_time
                        target2_id = -1
    log.write(f"{robot_id, target0_id, target1_id, target2_id}\n")            

    robots[robot_id].value = best_val_time
    return robot_id, target0_id, target1_id, target2_id


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


def bfs_init():
    global env_mp, DIS_MP, workbench_taking_mp, workbench_nothing_mp, dis_taking_mp, dis_nothing_mp, index_taking_mp, index_nothing_mp
    # workbench_taking_mp[i]表示工作台i的带东西bfs地图, workbench_nothing_mp[i]表示工作台i的不带东西的bfs地图
    # bfs返回的是一个map
    workbench_taking_mp = np.full((workbench_ids, cfg.MAP_SIZE, cfg.MAP_SIZE, cfg.MAX_ANS_MP_SIZE, 2), -1, dtype = int)
    workbench_nothing_mp = np.full((workbench_ids, cfg.MAP_SIZE, cfg.MAP_SIZE, cfg.MAX_ANS_MP_SIZE, 2), -1, dtype = int)
    index_taking_mp = np.zeros((workbench_ids, cfg.MAP_SIZE, cfg.MAP_SIZE), dtype = int)
    index_nothing_mp = np.zeros((workbench_ids, cfg.MAP_SIZE, cfg.MAP_SIZE), dtype = int)

    for id in range(workbench_ids):
        # log.write(f'{anti_cal_x(workbenchs[id].x)},{anti_cal_y(workbenchs[id].y)}')
        workbench_taking_mp_temp, index_taking_mp_temp = bfs_np(env_mp, (anti_cal_x(workbenchs[id].x), anti_cal_y(workbenchs[id].y)), 1)
        # log.write(f'{workbench_taking_mp}\n {np.array([workbench_taking_mp_temp])}\n')
        workbench_taking_mp[id] = workbench_taking_mp_temp
        index_taking_mp[id] = index_taking_mp_temp

        workbench_nothing_mp_temp, index_nothing_mp_temp = bfs_np(env_mp, (anti_cal_x(workbenchs[id].x), anti_cal_y(workbenchs[id].y)), 0)
        workbench_nothing_mp[id] = workbench_nothing_mp_temp
        index_nothing_mp[id] = index_nothing_mp_temp

    # dis_taking_mp[id0][id1]表示工作台id0和工作台id1之间的距离, all_taking_m[id0][id1]表示工作台id0和工作台id1之间的路径
    # log.write(f"{workbench_taking_mp[0][anti_cal_x(workbenchs[19].x)][anti_cal_y(workbenchs[19].y)]}\n")
    # log.write(f"{path_better(env_mp,[(47, 5), (46, 5), (45, 5), (44, 5), (44, 6), (43, 6), (42, 6), (41, 6), (40, 6), (39, 6), (38, 6), (37, 6), (36, 6), (35, 6), (34, 6), (33, 6), (32, 6), (31, 6), (30, 6), (29, 6), (28, 6), (27, 6), (26, 6), (25, 6), (24, 6), (23, 6), (22, 6), (21, 6), (20, 6), (19, 6), (18, 6), (17, 6), (16, 6), (15, 6), (14, 6), (14, 7), (14, 8), (14, 9), (14, 10), (14, 11), (14, 12), (14, 13), (14, 14), (14, 15), (14, 16), (14, 17), (14, 18), (14, 19), (14, 20), (14, 21), (14, 22), (14, 23), (14, 24), (14, 25), (14, 26), (14, 27), (14, 28), (14, 29), (14, 30), (14, 31), (14, 32), (14, 33), (14, 34), (14, 35), (14, 36), (14, 37), (14, 38), (14, 39), (14, 40), (14, 41), (14, 42), (14, 43), (14, 44), (14, 45), (14, 46), (14, 47), (14, 48)], 1)}\n")

    for id0 in range(workbench_ids):
        for id1 in range(id0 + 1, workbench_ids):
            id0_x, id0_y = anti_cal_x(workbenchs[id0].x), anti_cal_y(workbenchs[id0].y)
            id1_x, id1_y = anti_cal_x(workbenchs[id1].x), anti_cal_y(workbenchs[id1].y)
            dis_taking_mp[id0][id1] = dis_taking_mp[id1][id0] = index_taking_mp[id0][id1_x][id1_y]
            dis_nothing_mp[id0][id1] = dis_nothing_mp[id1][id0] = index_nothing_mp[id0][id1_x][id1_y]
            # log.write(f'workbench_taking_mp[id0][id1_x][id1_y]:{workbench_taking_mp[id0][id1_x][id1_y]}\n')
            all_taking_mp[id0][id1] = path_better_np(env_mp, workbench_taking_mp[id0][id1_x][id1_y][:index_taking_mp[id0][id1_x][id1_y]], 1)
            all_nothing_mp[id0][id1] = path_better_np(env_mp, workbench_nothing_mp[id0][id1_x][id1_y][:index_nothing_mp[id0][id1_x][id1_y]], 0)

    log.write(f"{(anti_cal_x(workbenchs[0].x), anti_cal_y(workbenchs[0].y))} {(anti_cal_x(workbenchs[19].x), anti_cal_y(workbenchs[19].y))}\n")
    log.write(f"dis_taking_mp:\n{dis_taking_mp[0][19]}\n")
    log.write(f"dis_nothing_mp:\n{dis_nothing_mp[0][19]}\n")
    log.write(f"all_taking_mp:\n{all_taking_mp[0][19]}\n")
    log.write(f"all_nothing_mp:\n{all_nothing_mp[0][19]}\n")
    exit()



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