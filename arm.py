# ARM OBJECT

class Arm:
    def __init__(self, name, velocity, position, destination):
        self.name = name
        self.vel = velocity
        self.pos = position
        self.dest = destination

    def get_name(self):
        return self.name

    def get_position(self):
        return self.pos

    def set_position(self, position):
        self.pos = position

    def get_destination(self):
        return self.dest

    def set_destination(self, destination):
        self.dest = destination