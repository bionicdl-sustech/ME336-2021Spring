# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/9/23
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""

import numpy as np
import time
import sys
import os
import yaml

# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
sys.path.append(_root_path)
# os.chdir(_root_path)

from deepclaw.driver.sensors.camera.AzureKinect_pyk4a import AzureKinect
camera = AzureKinect()
time.sleep(1)
frame = camera.get_frame()
pcs = frame.point_cloud[0]
depth_img = frame.depth_image[0]
crop_bounding = [200, 470, 360, 720]
# transfer to point cloud
width = 1280
hight = 720
fx = 602.365
fy = 602.393
cx = 636.734
cy = 363.352
pc = []
for i in range(crop_bounding[1], crop_bounding[3]):
    temp_pc = []
    for j in range(crop_bounding[0], crop_bounding[2]):
        z = depth_img[i][j]
        x = (j - cx) * z / fx
        y = (i - cy) * z / fy
        if z == 0:
            temp_pc.append([np.nan, np.nan, np.nan])
        else:
            temp_pc.append([x, y, z])
    pc.append(temp_pc)
pc = np.array(pc).reshape((-1, 3))
# filter plane
plane_model = [-0.07, 0.25, 0.97, -0.98]
new_pc = []
for i in range(len(pc)):
    if pc[i][0]*plane_model[0] + pc[i][1]*plane_model[1] + pc[i][2]*plane_model[2] + plane_model[3] < 0:
        new_pc.append(pc[i])

new_pc = np.array(new_pc)


import GeoGrasp
import numpy as np

m = GeoGrasp.run(new_pc)
m = np.array(m)
# row
k = np.reshape(m, (4, 4), order='C')
print(k)
# from matplotlib import pyplot as plt
# fig = plt.figure("pcs")
# ax = fig.add_subplot(111, projection="3d")
# ax.scatter(pcs[:, 0], pcs[:, 1], pcs[:, 2], s=1,)
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
# ax.set_xlim(-1, 1)
# ax.set_ylim(-1, 1)
# ax.set_zlim(0.5, 2)
# ax.view_init(elev=-90, azim=-90)
# plt.show()
