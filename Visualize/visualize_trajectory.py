#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
# read the trajectory from the file on the build directory ../build/lqr_speed_control.csv
trajectories = []
current_dir = os.path.dirname(os.path.abspath(__file__))
with open(f'lqr_speed_control_path.csv', 'r') as file:
    # each row is comma separated values where x and y are on the second and third columns
    trajectory = np.loadtxt(file, delimiter=',')
   

# rint original path
with open(f'path.csv', 'r') as file:
    path = np.loadtxt(file, delimiter=',')
    
plt.plot(path[:, 0], path[:, 1],linewidth=5, color='black')
plt.plot(trajectory[:,1], trajectory[:, 2],linewidth=1, color='red')
plt.show()

# plot the planned curvature
plt.figure()
plt.plot(path[:,2],linewidth=5, color='black')
plt.show()
plt.title('Planned Heading')
plt.xlabel('Time step')
plt.ylabel('Heading')

# plot the taken curvature
plt.figure()
plt.plot(trajectory[:,3],linewidth=1, color='red')
plt.show()
plt.title('Taken Heading')
plt.xlabel('Time step')
plt.ylabel('Heading')