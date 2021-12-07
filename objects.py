import numpy as np

# PICK AND PLACE OBJECTS

class Object:
    def __init__(self, name, position, color=None, plot=None):
        self.name = name
        self.pos = np.array(position)
        self.plot = plot
        self.color = color

    def get_name(self):
        return self.name

    def get_position(self):
        return self.pos

    def get_plot(self):
        return self.plot

    def set_plot(self, plot):
        self.plot = plot
    
    def get_color(self):
        return self.color

    def set_color(self, color):
        self.color = color