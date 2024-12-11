# plot the speed profile
import matplotlib.pyplot as plt
import numpy as np

speed_profile = np.loadtxt('build/speed_profile.csv', delimiter=',')
plt.plot(speed_profile)
plt.show()
