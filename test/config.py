'''
Author: BigCiLeng w15516937650@outlook.com
Date: 2023-03-22 16:27:44
LastEditors: BigCiLeng && bigcileng@outlook.com
LastEditTime: 2023-04-08 22:22:27
FilePath: \HUAWEIRobot\test\config.py
Description: 

Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
'''
import math

class CFG():
    ROBOT_RADIUS = 0.45
    ROBOT_RADIUS_MAX = 0.53
    ROBOT_RHO = 20
    ROBOT_NUM = 4
    MAP_SIZE = 100
    PI = math.pi
    THING_VALUE = [0, 3000, 3200, 3400, 7100, 7800, 8300, 29000]
    THING_VALUE_2 = [0, 3000, 3100, 3200, 8100, 8200, 8300, 29000]
    BATTLE_TIME = [50, 100, 100, 100, 150, 150, 150, 200]
    THING_COEFF = [0, 1/6, 1/6, 1/6, 1/3, 1/3, 1/3, 1]
    HIGH_LEVEL_WORKBENCH = [4, 5, 6, 7]
    USEFUL_WORKBENCH = [1, 2, 3, 4, 5, 6, 7]
    MAX_WAIT_TIME = 50
    MAX_PENTALIY_VALUE = 100000
    OVER_FRAME = 9000
    SUB_MISSION = 30000
    TASK_TYPE = [7, 6, 5, 4, 3, 2, 1]
    DIS_NORMAL = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    DIS_HIGHER = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    DIS_HALF = [(0.5, 0), (-0.5, 0), (0, 0.5), (0, -0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, 0.5), (-0.5, -0.5)]
    gvalue = [10, 10, 10, 10, 14, 14, 14, 14]
    MAP_SIZE_2 = 200
    MAX_ANS_MP_SIZE = 500                           

    ###
    # use for orca
    dt = 1 / 50
    tau = 25 * dt
    pid_list = [[0, 0], [0, 0], [0, 0], [0, 0]]
    ###
    temp_flag = 0

    MAX_STOP_DISTANCE_0 = 0.9160884177867836 + 0.4
    MAX_STOP_DISTANCE_1 = 1.270761662006457 + 0.4
    DIS_TOLERATE = 0.2
    BACK_DISTANCE = 10
