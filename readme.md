# HUAWEURobot

## 机器人类

```python
# 变量
self.work_space = -1
self.take_thing = 0
self.time_f = 0.0
self.crush_f = 0.0
self.angle_speed = 0.0
self.line_speed_x, self.line_speed_y = 0.0, 0.0
self.toward = 0.0
self.x, self.y = 0.0, 0.0
self.robot_id = id
self.work_frame = 1
self.state = 0
self.target_workbench_ids=-1
self.s_pid=PID.PID(10, 0.01, 300, 0)
self.w_pid=PID.PID(10, 0.01, 3, 0)
# 函数
move_to_target(self, direction, distance)   # pid控制机器人
```

## 工作台类

```python
# 变量
self.work_type = 0  # 工作台类型
self.x, self.y = 0.0, 0.0
self.remain_time = -1
self.origin_thing =0
self.output = 0
self.table_id = id

# the workbench is targeted by a robot
self.is_targeted_flag = 0
# 函数
```

## 全局变量

```python
dis_mp  # 两个workbench之间的距离
workbench_xynum_dist    # 通过坐标获取workbench id
workbenchs  # 工作台集合
robots  # 机器人集合
workbench_type_num # 每种workbench包含的工作台ids集合
```

## 全局函数

```python
cal_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y)  # x,y之间距离
drt_point_x_y(origin_x: float, origin_y: float, target_x: float, target_y)  # x到y的方向角
find_free_robot(robots) # 没有目标的机器人的id集合
find_free_job(workbenchs)   # 空闲任务的字典集合,key为工作台id,value为任务类型,按value从大到小排序
get_job() # 将空闲任务按性价比分配给空闲机器人
find_nearest_target(x, y, workbenchs, target_workbench_list, workbench_type_num, take_thing)    # 寻找target_list里最近的可用的目标（如果take_thing且卖不出去，则不可用
```
