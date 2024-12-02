#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# read the path from the csv file
path = np.loadtxt('build/path.csv', delimiter=',')
plt.plot(path[:, 0], path[:, 1])
plt.show()