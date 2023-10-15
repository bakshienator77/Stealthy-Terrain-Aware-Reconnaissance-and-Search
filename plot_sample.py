"""
Code for the work:

`` Stealthy Terrain-Aware Multi-Agent Active Search``,
Nikhil Angad Bakshi and Jeff Schneider
Robotics Institute, Carnegie Mellon University

(C) Nikhil Angad Bakshi 2023 (nabakshi@cs.cmu.edu)
Please cite the following paper to use the code:


@inproceedings{
bakshi2023stealthy,
title={Stealthy Terrain-Aware Multi-Agent Active Search},
author={Nikhil Angad Bakshi and Jeff Schneider},
booktitle={7th Annual Conference on Robot Learning},
year={2023},
url={https://openreview.net/forum?id=eE3fsO5Mi2}
}
"""

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import numpy as np

from mpl_toolkits.mplot3d import axes3d, Axes3D 
fig = plt.figure()
ax = Axes3D(fig)

# Make data.
X = np.arange(0, 20, 2*0.125)
Y = np.arange(0, 20, 2*0.125)
X, Y = np.meshgrid(X, Y)

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
kernel = 1 * RBF(length_scale=3, length_scale_bounds=(1e-2, 1e2))
gaussian_process = GaussianProcessRegressor(kernel=kernel, optimizer=None, n_restarts_optimizer=1)
gaussian_process.fit(np.asarray([[4, 4]]), np.asarray([1]))

Z = gaussian_process.predict(np.concatenate((X.reshape(-1, 1), Y.reshape(-1, 1)), axis=1)).reshape((80, 80))

# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Customize the z axis.
ax.set_zlim(-1.01, 1.01)
ax.zaxis.set_major_locator(LinearLocator(10))

plt.axis('off')

# Add a color bar which maps values to colors.
#fig.colorbar(surf, shrink=0.5, aspect=5)

#plt.show()
plt.savefig("reward.png", transparent=True, dpi=1200)