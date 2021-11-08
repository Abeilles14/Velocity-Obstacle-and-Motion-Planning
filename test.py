import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, ax = plt.subplots()

g = 9.8                                                        #value of gravity
v = 10.0                                                       #initial velocity
theta = 40.0 * np.pi / 180.0                                   #initial angle of launch in radians
t = 2 * v * np.sin(theta) / g                                  
t = np.arange(0, 0.1, 0.01)                                    #time of flight into an array
x = np.arange(0, 0.1, 0.01)
line, = ax.plot(x, v * np.sin(theta) * x - (0.5) * g * x**2)   # plot of x and y in time

def animate(i):
    """change the divisor of i to get a faster (but less precise) animation """
    line.set_xdata(v * np.cos(theta) * (t + i /100.0))
    line.set_ydata(v * np.sin(theta) * (x + i /100.0) - (0.5) * g * (x + i / 100.0)**2)  
    return line,

plt.axis([0.0, 10.0, 0.0, 5.0])
ax.set_autoscale_on(False)

ani = animation.FuncAnimation(fig, animate, np.arange(1, 200))
plt.show()