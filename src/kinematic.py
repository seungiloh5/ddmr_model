"""

Kinematic modelling of differential drive mobile robot

author: Seung Il Oh

"""

import Tkinter
import matplotlib 
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math
import time
from drawnow import *

def kinematic_equation():
    # initial conditions
    x0 = 0
    y0 = 0
    theta0 = 0

    # parameters
    global r, b
    r = 0.075 # wheel raduis
    b = 0.15  # half of the wheel-to-wheel distance

    # time vector and time increment
    global m, t
    tf = 3
    m = 60
    t = np.linspace(0,tf,m)
    dt = round(t[1]- t[0],4)

    # initialization and pre-allocation of pose variables
    x = np.zeros(len(t))
    y = np.zeros(len(t))
    theta = np.zeros(len(t))
    x[0] = x0
    y[0] = y0
    theta[0] = theta0

    # input signals (in RPM)
    n = 20
    rpm = [240*(2*math.pi/60), 120*(2*math.pi/60), -240*(2*math.pi/60), -120*(2*math.pi/60)] 
    vec = np.ones(n)
    phidot_r = np.concatenate([vec*rpm[0], vec*rpm[1], vec*rpm[1]])
    phidot_l = np.concatenate([vec*rpm[1], vec*rpm[0], vec*rpm[3]])

    # update loop
    for ii in range(1,m):
        A = np.array([[r*math.cos(theta[ii-1])/2, r*math.cos(theta[ii-1])/2], \
                        [r*math.sin(theta[ii-1])/2,r*math.sin(theta[ii-1])/2], \
                        [r/(2*b), -r/(2*b)]])
        phidot = np.array([[phidot_r[ii-1]],[phidot_l[ii-1]]])
        deltapose = dt*np.dot(A,phidot)
        x[ii] = x[ii-1]+deltapose[0]
        y[ii] = y[ii-1]+deltapose[1]
        theta[ii] = theta[ii-1]+deltapose[2]
    return x, y, theta, phidot_r, phidot_l

def update(i):
    wlsr = np.array([[math.cos(theta[i]), -math.sin(theta[i])], [math.sin(theta[i]), math.cos(theta[i])]])
    wlsrot = np.dot(wlsr, wls)
    # print(wlsrot)
    wheel1.set_data(wlsrot[0][0]+x[i], wlsrot[1][0]+y[i])
    wheel2.set_data(wlsrot[0][1]+x[i], wlsrot[1][1]+y[i])
    chassis.set_data(x[i],y[i])

    return line, wheel1, wheel2, chassis,

if __name__ == '__main__':
    
    x, y, theta, phidot_r, phidot_l = kinematic_equation()

    # plot and animation
    fig1, axes = plt.subplots(2,1)
    axes[0].plot(t,phidot_r, label='right wheel')
    axes[0].hold(True)
    axes[0].plot(t,phidot_l, label='left wheel')
    axes[0].set_ylabel('[input] angular vel. of the wheels')
    axes[0].legend()
    axes[0].grid(True)
    axes[1].plot(t,theta)
    axes[1].set_xlabel('time')
    axes[1].set_ylabel('angle of the robot chassis')

    fig2, ax = plt.subplots()
    line, =  ax.plot(x,y,'b--',label='robot route')
    wheel1, = ax.plot([], [], 'ro')
    wheel2, = ax.plot([], [], 'ro')
    chassis, = ax.plot([], [], marker='o',ms='10',mec='r',mfc='w')

    wls = np.array([[0, 0], [-0.5*b, 0.5*b]])
    ani = FuncAnimation(fig2, update, frames=range(0,m))
    plt.show()