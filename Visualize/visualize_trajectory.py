#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# read the trajectory from the file on the build directory ../build/lqr_speed_control.csv
trajectories = []
with open(f'build/lqr_speed_control_path.csv', 'r') as file:
    # each row is comma separated values where x and y are on the second and third columns
    trajectory = np.loadtxt(file, delimiter=',', usecols=(1, 2))
print(trajectory)



# print original path
with open(f'build/path.csv', 'r') as file:
    path = np.loadtxt(file, delimiter=',', usecols=(0, 1))
print(path)
plt.plot(path[:, 0], path[:, 1],linewidth=5, color='black')
plt.plot(trajectory[:, 0], trajectory[:, 1],linewidth=1, color='red')
plt.show()
