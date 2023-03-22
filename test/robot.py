import PID
import math
import sys

PI = math.pi


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
        self.task_by_time = 0.0
        self.work_frame = 1
        self.state = 0
        # next target workbench
        self.target_workbench_ids = [-1, -1]
        self.s_pid=PID.PID(50, 0.001, 1300, 0)
        # 0.8 0.005
        self.w_pid=PID.PID(50, 0.01, 3, 0)

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
                tolerance0=0.001, tolerance1=0.4):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if abs(steering) < tolerance0:
            steering = 0
        if abs(distance) < tolerance1:
            distance = 0
        
        # apply pid
        sys.stdout.write('rotate %d %f\n' % (self.robot_id, steering))

        if distance < 0:
            sys.stdout.write('forward %d %f\n' % (self.robot_id, -2))
        else:
            speed = min(6, distance)
            sys.stdout.write('forward %d %f\n' % (self.robot_id, speed))

    def move_to_target(self, direction, distance):
        direction1=direction-self.toward
        if direction1 > PI:
            direction1 += -2*PI
        elif direction1 <= -PI:
            direction1 += 2*PI

        DISTANCE_TOLERATION = 2

        # 左墙面 -->右墙面 -->上墙面 -->下墙面
        if ((self.x <= DISTANCE_TOLERATION) and (self.toward >= PI * 3 / 4 or self.toward <= -PI * 3 / 4)) \
            or ((self.x >= (50 - DISTANCE_TOLERATION)) and (self.toward >= -PI / 4 and self.toward <= PI / 4)) \
            or ((self.y >= (50 - DISTANCE_TOLERATION)) and (self.toward > PI / 4 or self.toward < PI * 3 / 4)) \
            or ((self.y <= DISTANCE_TOLERATION) and (self.toward > -PI * 3 / 4 and self.toward < -PI / 4)):
            if direction1 > PI / 2 or direction1 < -PI / 2:
                distance = - distance
        
        steering = self.w_pid.control(-direction1)
        move_distance = self.s_pid.control(-distance)
        # log.write(f'{self.s_pid.accumulated_error} {self.s_pid.previous_error}\n')
        self.move(steering, move_distance)