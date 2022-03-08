from scipy.integrate import solve_ivp
import Tkinter
import matplotlib 
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import numpy as np


print(np.zeros(10))
# def exponential_decay(t, y):
#      return -0.5 * y

# sol = solve_ivp(exponential_decay, [0, 10], [2, 4, 8])


# fig1, axes = plt.subplots(3,1)
# axes[0].plot(sol.t, sol.y[0,:])
# axes[1].plot(sol.t, sol.y[1,:])
# axes[2].plot(sol.t, sol.y[2,:])
# plt.show()

# M = np.array([ [5, 0, 3, 0, 0], \
#             [0, 2, 5, 0, 0], \
#             [1, 5, 7, 0, 0], \
#             [0, 0, 0, 2, 0], \
#             [0, 0, 0, 0, 2] ])

# print(M)

# M_inv = np.linalg.inv(M)

# print(M_inv)

# M_transpose = np.transpose(M)

# print(M_transpose)

