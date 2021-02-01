# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/25
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: create grasping data from image masks
"""

import glob
import json

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# x,y = np.mgrid[-5:5:200j,-5:5:200j]
# sigma = 2
# z = 1/(2 * np.pi * (sigma**2)) * np.exp(-(x**2+y**2)/(2 * sigma**2))
#
#
# fig = plt.figure()
# ax = Axes3D(fig)
# ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='rainbow',alpha = 0.9)
#
#
#
# plt.show()

mean = [0, 0]
cov = [[10, 0], [0, 100]]  # covariance
x, y = np.random.multivariate_normal(mean, cov, 5000).T
plt.plot(x, y, 'x')
plt.axis('equal')
plt.show()


np.random.seed(20)
random6 = np.random.rand(3,3)
print(random6)
