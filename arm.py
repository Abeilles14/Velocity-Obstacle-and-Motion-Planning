# ARM OBJECT

class Arm:
    def __init__(self, name, velocity, position):
        self.name = name
        self.vel = velocity
        self.pos = position

    def get_name(self):
        return self.name

    def position(self):
        return self.position