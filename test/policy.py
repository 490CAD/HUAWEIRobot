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
