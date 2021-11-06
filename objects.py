# PICK AND PLACE OBJECTS

class Object:
    def __init__(self, name, arm, start, goal):
        self.name = name
        self.arm = arm
        self.start = start
        self.goal = goal

    def get_name(self):
        return self.name

    def position(self):
        return self.start

    def goal(self):
        return self.goal