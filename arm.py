# ARM OBJECT

class Arm:
    def __init__(self, name, home, position, destination, velocity):
        self.name = name
        self.home = home
        self.vel = velocity
        self.pos = position
        self.dest = destination
        self.vel = velocity

    def get_name(self):
        return self.name

    def get_home(self):
        return self.home

    def get_position(self):
        return self.pos

    def set_position(self, position):
        self.pos = position

    def get_destination(self):
        return self.dest

    def set_destination(self, destination):
        self.dest = destination

    def get_velocity(self):
        return self.vel

    def set_velocity(self, velocity):
        self.vel = velocity