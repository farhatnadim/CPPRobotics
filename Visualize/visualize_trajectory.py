import numpy as np
import matplotlib.pyplot as plt

# read the path from the csv file
path = np.loadtxt('build/path.csv', delimiter=',')
trajectory = np.loadtxt('build/lqr_speed_control_0.csv', delimiter=',')
plt.plot(path[:, 0], path[:, 1],linewidth=1, color='black')
# now read the trajectory from the csv file
print(trajectory)
plt.plot(trajectory[:, 1], trajectory[:, 2],linewidth=5, color='red')
plt.show()
# in a new figure, read the speed profile from the csv file
speed_profile = np.loadtxt('build/speed_profile.csv', delimiter=',')
plt.figure()
plt.plot(speed_profile)
plt.show()