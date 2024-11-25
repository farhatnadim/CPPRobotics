#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# read the trajectory from the file on the build directory ../build/lqr_speed_control.csv
trajectories = []
for i in range(6):
    with open(f'build/lqr_speed_control_{i}.csv', 'r') as file:
        # each row is comma separated values where x and y are on the second and third columns
        trajectory = np.loadtxt(file, delimiter=',', usecols=(1, 2))
        # combine them into one figure
        trajectories.append(trajectory)

# plot all trajectories
for trajectory in trajectories:
    plt.plot(trajectory[:, 0], trajectory[:, 1])

plt.show()
