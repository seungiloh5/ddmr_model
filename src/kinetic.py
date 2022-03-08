"""

Kinetic modelling of differential drive mobile robot

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
from scipy.integrate import solve_ivp

def dynamic_equation(time,z):

    """
    z = [x, xdot, y, ydot, theta, thetadot, phi1, phi1dot, phi2, phi2dot]
    zdot = [xdot, xdotdot, ydot, ydotdot, thetadot, thetadotdot, phi1dot, phi1dotdot, phi2dot, phi2dotdot]
    """


    # generalized coordinates
    zdot = np.zeros(10)
    theta = z[4]
    thetadot = z[5]
    qdot = [ [z[1]], [z[3]], [z[5]], [z[7]], [z[9]] ]
    # parameters
    mW = 0.2 # mass of wheels
    mB = 3 # mass of the chassis
    mT = mB + 2*mW
    d = 0.1 # distance from the center of the wheels to the center of mass of chassis
    W = 0.15 # half of the wheel-to-wheel distance
    rho = 0.5 # radius of the wheels
    t = 0.01 # thickness of the wheels

    # moment of inertia
    IB = 0.5*mB*W**2 # moment of inertia of the chassis (depends on object shape)
    Iyy = mW*(rho**2)/2 # moment of inertia of wheels about the y-axis
    Izz = mW*(3*rho**2+t**2)/12 # moment of inertia of wheels about the z-axis
    IT = IB+mB*d**2+2*mW*W**2+2*Izz # total moment of inertia

    # input Torques
    T1 = 10
    T2 = 10

    # matrices for dynamic systems
    M = np.array([ [mT, 0, -mB*d*math.sin(theta), 0, 0], \
                [0, mT, mB*d*math.cos(theta), 0, 0], \
                [-mB*d*math.sin(theta), mB*d*math.cos(theta), IT, 0, 0], \
                [0, 0, 0, Iyy, 0], \
                [0, 0, 0, 0, Iyy] ])

    B = np.array([ [math.cos(theta)], [math.sin(theta)], [0], [0], [0] ])
    B = -mB*d*thetadot**2*B

    C = np.array([ [math.cos(theta), math.sin(theta), 0, rho/2, -rho/2], \
                [-math.sin(theta), math.cos(theta), 0, 0, 0], \
                [0, 0, 1, 0.5*rho/W, 0.5*rho/W] ])

    Cdot = np.array([ [-math.sin(theta)*thetadot, -math.cos(theta)*thetadot, 0, 0, 0], \
                    [-math.cos(theta)*thetadot, -math.sin(theta)*thetadot, 0, 0, 0], \
                    [0, 0, 0, 0, 0] ])

    T = np.array([ [0], [0], [0], [T1], [T2] ])

    # determine lambdas
    partA = -np.linalg.inv(np.matmul(np.matmul(C,np.linalg.inv(M)),np.transpose(C)))
    partB = np.matmul(np.matmul(C,np.linalg.inv(M)),(T-B))
    partC = np.matmul(Cdot,qdot)
    lambdas = np.matmul(partA,(partB + partC))

    # determine qdoubledot
    partD = np.linalg.inv(M)
    partE = T-B
    partF = np.matmul(np.transpose(C),lambdas)
    qdoubledot = np.matmul(partD,(partE+partF))

    zdot[0] = z[1]
    zdot[1] = qdoubledot[0]
    zdot[2] = z[3]
    zdot[3] = qdoubledot[1]
    zdot[4] = z[5]
    zdot[5] = qdoubledot[2]
    zdot[6] = z[7]
    zdot[7] = qdoubledot[3]
    zdot[8] = z[9]
    zdot[9] = qdoubledot[4]

    return np.array(zdot)

def update(i):
    wlsr = np.array([[math.cos(theta[i]), -math.sin(theta[i])], [math.sin(theta[i]), math.cos(theta[i])]])
    wlsrot = np.dot(wlsr, wls)
    wheel1.set_data(wlsrot[0][0]+x_axis[i], wlsrot[1][0]+y_axis[i])
    wheel2.set_data(wlsrot[0][1]+x_axis[i], wlsrot[1][1]+y_axis[i])
    chassis.set_data(x_axis[i],y_axis[i])
    return line, wheel1, wheel2, chassis,


if __name__ == '__main__':

    # set initial conditions (1: right, 2: left)
    x0 = 1.0
    xdot0 = 1.0
    y0 = 1.0
    ydot0 = 1.0
    theta = 1.0
    thetadot0 = 1.0
    phi10 = 1.0 
    phi1dot0 = 1.0
    phi20 = 1.0
    phi2dot0 = 1.0
    init_cond = np.array([x0, xdot0, y0, ydot0, theta, thetadot0, phi10, phi1dot0, phi20, phi2dot0])

    # set time
    time_span = np.array([0, 2])
    times = np.linspace(time_span[0], time_span[1], 201)

    sol = solve_ivp(dynamic_equation, time_span, init_cond, t_eval=times)
    # sol = solve_ivp(dynamic_equation, time_span, init_cond)

    print("completed")

    tt = sol.t
    x_axis = sol.y[0]
    y_axis = sol.y[2]
    theta = sol.y[4]
    phidot_r = sol.y[7]
    phidot_l = sol.y[9]
    
    # plot and animation
    fig1, axes = plt.subplots(2,1)
    axes[0].plot(tt,phidot_r, label='right wheel')
    axes[0].hold(True)
    axes[0].plot(tt,phidot_l, label='left wheel')
    axes[0].set_ylabel('angular vel. of the wheels')
    axes[0].legend()
    axes[0].grid(True)
    axes[1].plot(tt,theta)
    axes[1].set_xlabel('time')
    axes[1].set_ylabel('angle of the robot chassis')

    b = 0.15
    r = 0.075
    m = 100000
    fig2, ax = plt.subplots()
    line, =  ax.plot(x_axis,y_axis,'b--',label='robot route')
    wheel1, = ax.plot([], [], 'ro')
    wheel2, = ax.plot([], [], 'ro')
    chassis, = ax.plot([], [], marker='o',ms='10',mec='r',mfc='w')
    wls = np.array([[0, 0], [-0.5*b, 0.5*b]])
    ani = FuncAnimation(fig2, update, frames=range(0,m))
    plt.show()
