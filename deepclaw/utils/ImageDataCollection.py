# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/2/5
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: auto collect images(color, depth, infrared)
"""

import sys
import os
# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)
# print('work_dir: ', _root_path)

import time
import cv2
import numpy as np


class ImageCollection(object):
    """
    collect images
    the folder tree is:
    GraspingData
    ├── 20210125184650
    │   └── camera0
    │       ├── images
    │       │   ├── color
    │       │   ├── depth
    │       └─  └── infrared
    │  
    ├── 20210125185456
    │   └── camera0
    │       ├── images
    │       │   ├── color
    │       │   ├── depth
    │       └─  └── infrared

    """
    def __init__(self, *args):
        """ camera list.
        For example, c1 = camera.driver(cfg), c2 = camera.driver(cfg), ...
        xx = ImageCollection(c1, c2, ...) """
        self.camera = args

    def run(self, saved_folder_path='./projects/ICRA2020/', show_flag=False):
        """ record images"""
        print('camera num: ', len(self.camera[0]))
        folder_name = time.strftime("%Y%m%d%H%M%S", time.localtime())
        cnt = 0
        while True:
            for i in range(len(self.camera[0])):
                # creat folder for each camera
                clc_path = os.path.join(saved_folder_path, 'GraspingData', folder_name, 'camera%d'%i, 'images', 'color')
                dep_path = os.path.join(saved_folder_path, 'GraspingData', folder_name, 'camera%d'%i, 'images', 'depth')
                inf_path = os.path.join(saved_folder_path, 'GraspingData', folder_name, 'camera%d'%i, 'images', 'infrared')
                # creat image and label folder
                if not os.path.isdir(clc_path):
                    os.makedirs(clc_path)
                if not os.path.isdir(dep_path):
                    os.makedirs(dep_path)
                if not os.path.isdir(inf_path):
                    os.makedirs(inf_path)

                frame = self.camera[0][i].get_frame()
                color = frame.color_image[0]
                depth = frame.depth_image[0]
                infrared = frame.infrared_image
                # save image
                if len(infrared) == 1:
                    infd_name = "{:08d}".format(cnt) + '.jpg'
                    cv2.imwrite(inf_path + '/' + infd_name, infrared[0])
                elif len(infrared) == 2:
                    infd_L_name = "{:08d}".format(cnt) + '_L' + '.jpg'
                    infd_R_name = "{:08d}".format(cnt) + '_R_' + '.jpg'
                    cv2.imwrite(inf_path + '/' + infd_L_name, infrared[0])
                    cv2.imwrite(inf_path + '/' + infd_R_name, infrared[1])
                else:
                    print("Error of camera infrared images!")
                    sys.exit()

                bgr_name = "{:08d}".format(cnt) + '.jpg'
                # depth_name = "{:08d}".format(cnt) + '.npy'
                depth_name = "{:08d}".format(cnt) + '.pgm'
                cv2.imwrite(clc_path + '/' + bgr_name, color)
                # np.save(dep_path + '/' + depth_name, depth)
                cv2.imwrite(dep_path + '/' + depth_name, (depth*1000).astype(np.uint16))
                # show images
                if show_flag:
                    cv2.imshow('camera %d' % i, color)

            # show image
            cnt = cnt + 1

            key_num = cv2.waitKey(1)
            if key_num == 113:  # press 'q' for exit
                break
            elif key_num == 32:  # press space for pause
                print('Pause Program!')
                print("Press Enter/q to continue...")
                while 1:
                    recover_key = cv2.waitKey(100)
                    if recover_key == 13 or recover_key == 113:  # press enter/q for recovering
                        print("======== Recovered ========")
                        break

            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break


if __name__ == "__main__":
    from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
    c1 = Realsense('./configs/basic_config/camera_rs_d435.yaml')
    # c2 = Realsense('./configs/basic_config/camera_rs_d435_c2.yaml')
    # test = ImageCollection(c1, c2)
    test = ImageCollection([c1])
    test.run(saved_folder_path='./projects/ICRA2020/', show_flag=True)
