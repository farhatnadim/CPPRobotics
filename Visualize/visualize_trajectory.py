#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# read the trajectory from the file on the build directory ../build/lqr_speed_control.csv
trajectories = []
with open(f'build/lqr_speed_control_path.csv', 'r') as file:
    # each row is comma separated values where x and y are on the second and third columns
    trajectory = np.loadtxt(file, delimiter=',')
   

print(trajectory[:, 1],)



# print original path
with open(f'build/path.csv', 'r') as file:
    path = np.loadtxt(file, delimiter=',')
    
plt.plot(path[:, 0], path[:, 1],linewidth=5, color='black')
plt.plot(trajectory[:,1], trajectory[:, 2],linewidth=1, color='red')
plt.show()

# plot the planned curvature
plt.plot(path[:,3],linewidth=5, color='black')

# plot the taken curvature
plt.plot(trajectory[:,4],linewidth=1, color='red')
plt.show()
