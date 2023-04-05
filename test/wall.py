class Wall():
    def __init__(self, id, x, y):
        self.x, self.y = float(x), float(y)
        self.wall_id = id
    def get_from_frame(self, x, y):
        self.x, self.y = float(x), float(y)