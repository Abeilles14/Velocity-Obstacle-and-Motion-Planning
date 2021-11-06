# PICK AND PLACE OBJECTS

class Object:
    def __init__(self, name, arm, start):
        self.name = name
        self.arm = arm
        self.start = start

    def get_name(self):
        return self.name

    def position(self):
        return self.start