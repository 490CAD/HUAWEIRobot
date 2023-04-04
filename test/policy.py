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
def bfs_np(env_mp, st_pos, is_take_thing):
    # [ [x,y]  [is_take_thing, is_take_thing] [-1, -1] ]
    start = time.time()
    # st_pos = np.array(st_pos)
    q = deque()
    q.append((st_pos, is_take_thing, (-1, -1)))

    ans_mp = np.full((cfg.MAP_SIZE, cfg.MAP_SIZE), None)
    # ans_mp = np.array([np.array([np.array([]) for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE))])
    vis_mp = np.zeros((cfg.MAP_SIZE, cfg.MAP_SIZE), dtype = int)

    # ans_mp1 = [[[] for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
    # vis_mp1 = [[0 for i in range(cfg.MAP_SIZE)] for j in range(cfg.MAP_SIZE)]
    # log.write(f'ans_mp:\n{ans_mp}\nans_mp1:\n{ans_mp1}\n')
    # log.write(f'vis_mp:\n{vis_mp}\nvis_mp1:\n{vis_mp1}\n')
    
    # index_mp: 用于标记ans_mp更新到了第几步
    index_mp = np.zeros((cfg.MAP_SIZE, cfg.MAP_SIZE), dtype = int)
    vis_mp[st_pos[0]][st_pos[1]] = 1
    ans_mp[st_pos[0]][st_pos[1]] = st_pos
    # log.write(f'{index_mp}\n')
    index_mp[st_pos[0]][st_pos[1]] += 1
    log.write(f'init time:{time.time()- start}\n')
    while q:
        now_pos = q.popleft()
        for i, (nx, ny) in enumerate(cfg.DIS_NORMAL):
            # log.write(f'{now_pos}\n')
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
            # temp_ans = []
            # for point in ans_mp[now_pos[0][0]][now_pos[0][1]]:
            #     temp_ans.append(point)
            # temp_ans.append((nx, ny))
            vis_mp[nx][ny] = 1
            
            ans_mp[nx][ny] = now_pos[0]
            index_mp[nx][ny] = index_mp[now_pos[0][0]][now_pos[0][1]] + 1
            q.append(((nx, ny), is_taking, now_pos[0]))
    # log.write(f'{index_mp}\n')
    log.write(f'finish time:{time.time()- start}\n')
    # exit()
    return ans_mp, index_mp