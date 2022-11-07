class Coordinates:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class  QR:


    def __init__(self, n):
        self.is_initialized = False
        self.qr_coords = Coordinates(0, 0)
        self.world_coords = Coordinates(0, 0)
        self.id = n
        self.letter = ""

    def set_attributes(self, x, y, x_world, y_world, l):
        if self.is_initialized: return

        self.qr_coords = Coordinates(x, y)
        self.world_coords = Coordinates(x_world, y_world)
        self.letter = l


    def set_next_qr(self, x, y):
        self.qr_coords = Coordinates(x, y)

    def set_initialized(self):
        self.is_initialized = True

    def frame_conversion(self):
        return