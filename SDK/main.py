#!/bin/bash
import sys
import math


# hyperparameters
ROBOT_RADIUS = 0.45
ROBOT_RHO = 20
ROBOT_NUM = 4
MAP_SIZE = 100
PI = math.pi


# Robots and Craft Tables
class Robot():
    def __init__(self, id):
        self.work_space = -1
        self.take_thing = 0
        self.time_f = 0.0
        self.crush_f = 0.0
        self.angle_speed = 0.0
        self.line_speed_x, self.line_speed_y = 0.0, 0.0
        self.toward = 0.0
        self.x, self.y = 0.0, 0.0
        self.robot_id = id

    
    def get_from_frame(self, work_space, take_thing, time_f, crush_f, angle_speed, line_speed_x, line_speed_y, toward, x, y):
        self.work_space = int(work_space)
        self.take_thing = int(take_thing)
        self.time_f = float(time_f)
        self.crush_f = float(crush_f)
        self.angle_speed = float(angle_speed)
        self.line_speed_x, self.line_speed_y = float(line_speed_x), float(line_speed_y)
        self.toward = float(toward)
        self.x, self.y = float(x), float(y)



class Craft_Table():
    def __init__(self, id):
        self.work_type = 0
        self.x, self.y = 0.0, 0.0
        self.remain_time = -1
        self.origin_thing =0
        self.output = 0
        self.table_id = id

    def get_from_frame(self, work_type, x, y, remain_time, origin_thing, output):
        self.work_type = int(work_type)
        self.x, self.y = float(x), float(y)
        self.remain_time = int(remain_time)
        self.origin_thing = int(origin_thing)
        self.output = int(output)


# Calc Functions
def cal_x(row: int):
    return 0.25 + row * 0.50
    
def cal_y(col: int):
    # return 0.25 + col * 0.50
    return 50.00 - (0.25 + col * 0.50)
    
def cal_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y):
    return math.sqrt((origin_x - target_x) ** 2 + (origin_y - target_y) ** 2)


# Input and Output Functions 
def read_map():
    env_mp, row_cnt = [[] for i in range(MAP_SIZE)], 0
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


# Main
if __name__ == '__main__':
    # input env_map
    env_mp, dis_mp = read_map(), [[] for i in range(50)]
    finish()
    # init Craft_Table and robots
    craft_table_xynum_dist = {}
    craft_table_ids = 0
    craft_tables, robots = [], []
    craft_type_num = [[] for i in range(10)]
    # start working
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        # input every frame
        parts = line.split(' ')
        frame_id, money_id = int(parts[0]), int(parts[1])

        if frame_id == 0:
            # 0th frame use init
            craft_table_frame_num = int(input())
            for craft_table in range(craft_table_frame_num):
                craft_table_type, craft_table_x, craft_table_y, craft_table_remain, craft_table_origin, craft_table_output = input().split()
                # create a new craft table
                craft_table_xynum_dist[(float(craft_table_x), float(craft_table_y))] = craft_table_ids
                craft_tables.append(Craft_Table(craft_table_ids))
                craft_type_num[int(craft_table_type)].append(craft_table_ids)
                # init it 
                craft_tables[craft_table_ids].get_from_frame(craft_table_ids, craft_table_x, craft_table_y, craft_table_remain, craft_table_origin, craft_table_output)
                craft_table_ids += 1
            for robot in range(ROBOT_NUM):
                robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y = input().split()
                # create a new robot and init it
                robots.append(Robot(robot))
                robots[robot].get_from_frame(robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y)
            # init the distance tables
            for craft_a in range(0, craft_table_ids):
                for craft_b in range(craft_a + 1, craft_table_ids):
                    dis_mp[craft_a][craft_b] = dis_mp[craft_b][craft_a] = cal_point_x_y(craft_tables[craft_a].x, craft_tables[craft_a].y, craft_tables[craft_b].x, craft_tables[craft_b].y)
        else:
            # update
            craft_table_frame_num = int(input())
            for craft_table in range(craft_table_frame_num):
                craft_table_type, craft_table_x, craft_table_y, craft_table_remain, craft_table_origin, craft_table_output = input().split()
                craft_table_id = craft_table_xynum_dist[(float(craft_table_x), float(craft_table_y))]
                # update the craft table state
                craft_tables[craft_table_id].get_from_frame(craft_table_id, craft_table_x, craft_table_y, craft_table_remain, craft_table_origin, craft_table_output)
            for robot in range(ROBOT_NUM):
                robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y = input().split()
                # update the robot state
                robots[robot].get_from_frame(robot_work, robot_take, robot_time, robot_crush, robot_angle, robot_line_x, robot_line_y, robot_toward, robot_x, robot_y)
        line = input()
        finish()
        # do some operation
        sys.stdout.write('%d\n' % frame_id)
        line_speed, angle_speed = 3, 1.5
        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
            sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
        finish()
