import numpy as np

# PICK AND PLACE OBJECTS

class Object:
    def __init__(self, name, position):
        self.name = name
        self.pos = np.array(position)

    def get_name(self):
        return self.name

    def get_position(self):
        return self.pos