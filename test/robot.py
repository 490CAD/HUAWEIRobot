import PID
import math
import sys
import numpy as np
from collections import deque
from config import CFG
PI = math.pi
cfg = CFG()
# log = open("log.txt", "a")
class Robot():
    def __init__(self, id):
        """
        """
        self.work_space = -1
        self.take_thing = 0
        self.time_f = 0.0
        self.crush_f = 0.0
        self.angle_speed = 0.0
        self.line_speed_x, self.line_speed_y = 0.0, 0.0
        self.toward = 0
        self.x, self.y = 0.0, 0.0
        self.robot_id = id
        self.task_by_time = 0.0
        self.work_frame = 1
        self.state = 0
        # next target workbench
        self.target_workbench_ids = [-1, -1]
        # 50, 0.001, 1500, 0
        self.s_pid=PID.PID(7, 0, 0, 0)
        # 50, 0.01, 3, 0
        self.w_pid=PID.PID(27, 0, 0, 0)
        self.value = 0
        self.move_list_target0 = None
        self.move_list_target1 = None
        self.anti_x = None
        self.anti_y = None
        self.now_suppose_work_space = -1

        ## 回退
        self.move_history = deque()
        self.line_flag = 0
        ###

    def get_from_frame(self, work_space, take_thing, time_f, crush_f, angle_speed, line_speed_x, line_speed_y, toward, x, y):
        self.work_space = int(work_space)
        self.take_thing = int(take_thing)
        self.time_f = float(time_f)
        self.crush_f = float(crush_f)
        self.angle_speed = float(angle_speed)
        self.line_speed_x, self.line_speed_y = float(line_speed_x), float(line_speed_y)
        self.toward = float(toward)
        self.x, self.y = float(x), float(y)

    def move(self, steering, distance,
                tolerance0=0.001, tolerance1=0.2):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if abs(steering) < tolerance0:
            steering = 0
        if abs(distance) < tolerance1:
            distance = 0
        
        # apply pid
        rotate = steering

        if distance < 0:
            forward = max(-2, distance)
        else:
            # k = math.exp(cfg.MAX_STOP_DISTANCE_0) / 6 if self.take_thing == 0 else math.exp(cfg.MAX_STOP_DISTANCE_1) / 6
            
            # forward =  min(6, math.exp(distance) / k)
            # forward =  min(6, distance / 3)
            
            if distance >= 6:
                forward = 6
            else:
                forward = max(distance / 3, 1)
            # if distance >= 9:
            #     forward = 6
            # else:
            #     forward = 4 / (1 + math.exp(-15 * (distance - 8))) + 2
        if rotate > 0:
            rotate = min(PI, rotate)
        else:
            rotate = max(-PI, rotate)
        return rotate, forward

    def move_to_target(self, direction, distance, mode=1):
        direction1=direction-self.toward
        if direction1 > PI:
            direction1 += -2*PI
        elif direction1 <= -PI:
            direction1 += 2*PI

        ### 靠墙倒车
        # DISTANCE_TOLERATION = 2
        # # # 左墙面 -->右墙面 -->上墙面 -->下墙面
        # if ((self.x <= DISTANCE_TOLERATION) and (self.toward >= PI * 3 / 4 or self.toward <= -PI * 3 / 4)) \
        #     or ((self.x >= (50 - DISTANCE_TOLERATION)) and (self.toward >= -PI / 4 and self.toward <= PI / 4)) \
        #     or ((self.y >= (50 - DISTANCE_TOLERATION)) and (self.toward > PI / 4 or self.toward < PI * 3 / 4)) \
        #     or ((self.y <= DISTANCE_TOLERATION) and (self.toward > -PI * 3 / 4 and self.toward < -PI / 4)):
        #     if direction1 > PI / 2 or direction1 < -PI / 2:
        #         distance = - distance
        ###
        
        ### 倒车
        # if direction1 > PI / 2 or direction1 < - PI/2:
        #     distance = -distance
        # ###

        steering = self.w_pid.control(-direction1)
        move_distance = self.s_pid.control(-distance)
        # log.write(f'{self.s_pid.accumulated_error} {self.s_pid.previous_error}\n')
        return self.move(steering, move_distance)