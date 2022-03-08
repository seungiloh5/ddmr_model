import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
ax.set_xlim(0, 2*np.pi)
ax.set_ylim(-1.2, 1.2)

x, y = [], []
line, = ax.plot([], [], 'bo')
line2, = ax.plot([],[],'ro')


def update(frame):
    x.append(frame)
    y.append(np.sin(frame))
    x_p=frame
    y_p=np.sin(frame)
    print(x)
    print(y)
    line.set_data(x, y)
    line2.set_data(x_p,y_p)
    return line, line2,


ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 100))
plt.show()