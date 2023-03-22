class WorkBench():
    def __init__(self, id):
        self.work_type = 0
        self.x, self.y = 0.0, 0.0
        self.remain_time = -1
        self.origin_thing =0
        self.output = 0
        self.table_id = id

        # the workbench is targeted by a robot
        # 第0位为买锁，其余第i位为第i钟原料的锁
        self.is_targeted_flag = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    def get_from_frame(self, work_type, x, y, remain_time, origin_thing, output):
        self.work_type = int(work_type)
        self.x, self.y = float(x), float(y)
        self.remain_time = int(remain_time)
        self.origin_thing = int(origin_thing)
        self.output = int(output)