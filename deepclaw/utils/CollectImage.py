# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/12/14
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""

import sys
import os
import yaml

# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)
print('work_dir: ', _root_path)

from deepclaw.driver.sensors.camera.AzureKinect import AzureKinect
from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
import time
import cv2
import numpy as np


# set image folder
saved_folder = './saved_image'
if not os.path.exists(saved_folder):
    print("Create image folder")
    os.makedirs(saved_folder)

results_path = saved_folder + '/' + time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime(time.time()))
if not os.path.exists(results_path):
    os.makedirs(results_path)

# camera = AzureKinect()
camera = Realsense('./configs/basic_config/camera_rs.yaml')
time.sleep(1)

for i in range(0, 1000):
    """ get image"""
    frame = camera.get_frame()
    color_img = frame.color_image[0]
    depth_img = frame.depth_image[0]
    infrared_img = frame.infrared_image[0]
    clc_name = "c_" + "{:08d}".format(i) + '.jpg'
    depth_name = "d_" + "{:08d}".format(i) + '.npy'
    infrared_name = "i_" + "{:08d}".format(i) + '.jpg'

    cv2.imwrite(results_path + '/' + clc_name, color_img)
    np.save(results_path + '/' + depth_name, depth_img)
    cv2.imwrite(results_path + '/' + infrared_name, infrared_img)

    cv2.imshow(clc_name, color_img)
    cv2.waitKey()
    cv2.destroyAllWindows()
