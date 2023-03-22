'''
Author: BigCiLeng w15516937650@outlook.com
Date: 2023-03-22 16:27:44
LastEditors: BigCiLeng w15516937650@outlook.com
LastEditTime: 2023-03-23 00:07:08
FilePath: \WindowsRelease\test\config.py
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
    HIGH_LEVEL_WORKBENCH = [4, 5, 6, 7]
    USEFUL_WORKBENCH = [1, 2, 3, 4, 5, 6, 7]
    MAX_WAIT_TIME = 100
    MAX_PENTALIY_VALUE = 100000
    OVER_FRAME = 9000

    ###
    # use for orca
    tau = 50
    dt = 1 / 50
    pid_list = [[None, None], [None, None], [None, None], [None, None]]
    ###
    temp_flag = 0