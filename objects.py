# PICK AND PLACE OBJECTS

class Object:
    def __init__(self, name, arm, position):
        self.name = name
        self.arm = arm
        self.pos = position

    def get_name(self):
        return self.name

    def get_position(self):
        return self.pos