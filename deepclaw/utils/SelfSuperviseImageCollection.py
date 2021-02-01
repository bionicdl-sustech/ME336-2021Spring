# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/4
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
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
import json
import math
from sklearn.decomposition import PCA
from queue import Queue
from deepclaw.utils.ForegroundDetector import BackgroundDetector


class ImageCollection(object):
    """
    collect images and label them
    the folder tree is:
    GraspingData
    ├── 20210125184650
    │   └── camera0
    │       ├── images
    │       │   ├── color
    │       │   ├── depth
    │       │   └── infrared
    │       └── labels
    ├── 20210125185456
    │   └── camera0
    │       ├── images
    │       │   ├── color
    │       │   ├── depth
    │       │   └── infrared
    │       └── labels

    """
    def __init__(self, *args):
        """ camera list.
        For example, c1 = camera.driver(cfg), c2 = camera.driver(cfg), ...
        xx = ImageCollection(c1, c2, ...) """
        self.camera = args
        self.label_dict = {"version": [], "shapes": [], 'imagePath': [], "imageHeight": [], "imageWidth": []}
        self.shape_dict = {"label": [], "mask_index": [], "mask_path": [], "position": [], "angle": []}
        self.obj_class = "None"

        self.showed_clc = Queue()
        self.showed_clc_label = Queue()

    def setObjectClass(self, obj_class: str):
        """ set object class"""
        self.obj_class = obj_class

    def run(self, saved_folder_path='./projects/ICRA2020/', auto_label=True, region_parameter=[200, 1100, 0, 720], ref_img: list = [], camera_showed_id=-1):
        """ record images and auto labeling"""
        """ ref_img = ['./1.jpg', '2.jpg', '3.avi', ...] """
        print('camera num: ', len(self.camera[0]))
        folder_name = time.strftime("%Y%m%d%H%M%S", time.localtime())
        cnt = 0
        # ref image list
        if auto_label:
            ref_frame = []
            mask_detector = []
            for i in range(len(ref_img)):
                if ref_img[i] is None:
                    temp = BackgroundDetector()
                    mask_detector.append(temp)
                    ref_frame.append(None)
                elif ref_img[i].split('.')[-1] == 'avi':
                    # read video
                    temp = BackgroundDetector()
                    temp.multiFrameLoader(ref_img[i], ref_num=1000, mog_threshold=80)
                    mask_detector.append(temp)
                    ref_frame.append('video')
                else:
                    # read image
                    temp = BackgroundDetector()
                    mask_detector.append(temp)
                    temp_color = cv2.imread(ref_img[i])
                    ref_frame.append(temp_color)

        while True:
            for i in range(len(self.camera[0])):
                # creat folder for each camera
                clc_path = os.path.join(saved_folder_path, 'GraspingData', folder_name, 'camera%d'%i, 'images', 'color')
                dep_path = os.path.join(saved_folder_path, 'GraspingData', folder_name, 'camera%d'%i, 'images', 'depth')
                inf_path = os.path.join(saved_folder_path, 'GraspingData', folder_name, 'camera%d'%i, 'images', 'infrared')
                label_path = os.path.join(saved_folder_path, 'GraspingData', folder_name, 'camera%d'%i, 'labels')
                # creat image and label folder
                if not os.path.isdir(clc_path):
                    os.makedirs(clc_path)
                if not os.path.isdir(dep_path):
                    os.makedirs(dep_path)
                if not os.path.isdir(inf_path):
                    os.makedirs(inf_path)
                if not os.path.isdir(label_path):
                    os.makedirs(label_path)

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
                if camera_showed_id == -1 and not auto_label:
                    cv2.imshow('camera %d' % i, color)

                # save label
                if auto_label:
                    # mask for objects
                    if ref_frame[i] is None:
                        bi_mask = mask_detector[i].filterColor(color)
                    elif ref_frame[i] == 'video':
                        bi_mask = mask_detector[i].multiFrameFilter(color)
                    else:
                        bi_mask = mask_detector[i].diffGround(ref_frame[i], color)

                    # ROI
                    # region_parameter = [200, 1100, 0, 720]
                    height_row = color.shape[0]
                    width_col = color.shape[1]
                    region_mask = np.zeros((height_row, width_col), dtype=np.uint8)
                    image_1 = np.ones((height_row, width_col), dtype=np.uint8)
                    region_col_min = region_parameter[0]
                    region_col_max = region_parameter[1]
                    region_row_min = region_parameter[2]
                    region_row_max = region_parameter[3]
                    region_mask[region_row_min:region_row_max, region_col_min:region_col_max] = image_1[region_row_min:region_row_max, region_col_min:region_col_max]
                    bi_mask = np.multiply(bi_mask, region_mask)

                    # mask labels
                    labels, labels_index, color_labels = mask_detector[i].getConnectedDomain(bi_mask, show_label=False, region_area=2000)
                    # gui display
                    if camera_showed_id == i:
                        self.showed_clc.put(color)
                        self.showed_clc_label.put(color_labels)
                    # show image and mask
                    overlapping = cv2.addWeighted(color, 0.6, color_labels, 0.4, 0)
                    cv2.imshow('camera %d' % i, overlapping)

                    # save mask label
                    # seg_mask_name = "{:08d}".format(cnt) + '.npy'
                    seg_mask_name = "{:08d}".format(cnt) + '.pgm'
                    color_mask_name = "{:08d}".format(cnt) + '.jpg'
                    # np.save(label_path + '/' + seg_mask_name, labels)
                    cv2.imwrite(label_path + '/' + color_mask_name, color_labels)
                    cv2.imwrite(label_path + '/' + seg_mask_name, labels.astype(np.uint16))
                    self.label_dict["version"] = '4.5.6'
                    self.label_dict["imagePath"] = '../images/color/' + bgr_name
                    self.label_dict["imageHeight"] = color.shape[0]
                    self.label_dict["imageWidth"] = color.shape[1]
                    pick_temp = []
                    for k in range(len(labels_index)):
                        if labels_index[k] == 0:
                            continue
                        else:
                            self.shape_dict["label"] = self.obj_class
                            self.shape_dict["mask_index"] = labels_index[k]
                            self.shape_dict["mask_path"] = seg_mask_name
                            index_row, index_col = np.where(labels == labels_index[k])
                            x_data = np.zeros([len(index_row), 2])  # row, col
                            x_data[:, 1] = index_row
                            x_data[:, 0] = index_col
                            # grasping position and angle
                            pca = PCA(n_components=2)
                            pca.fit(x_data)
                            best_axis = pca.components_[0]  # 长轴方向
                            if best_axis[0] == 0:
                                theta = math.pi/2
                            else:
                                theta = math.atan(best_axis[1]/best_axis[0])  # (-pi/2, pi/2)

                            self.shape_dict["label"] = self.obj_class
                            self.shape_dict["position"] = pca.mean_.tolist()  # col, row
                            self.shape_dict["angle"] = theta
                            pick_temp.append(self.shape_dict.copy())

                    self.label_dict["shapes"] = pick_temp

                    labels_name = "{:08d}".format(cnt) + '.json'
                    with open(label_path + '/' + labels_name, 'w') as f:
                        json.dump(self.label_dict, f, indent=4)
            # show image
            cnt = cnt + 1

            key_num = cv2.waitKey(1)
            if key_num == 113:  # press 'q' for exit
                break
            elif key_num == 32:  # press space for pause
                print('Pause Program!')
                print("Press Enter to continue...")
                while 1:
                    recover_key = cv2.waitKey(100)
                    if recover_key == 13 or recover_key == 113:  # press enter/q for recovering
                        print("======== Recovered ========")
                        break

            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break


if __name__ == "__main__":
    from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
    c1 = Realsense('./configs/basic_config/camera_rs_d435_c1.yaml')
    # c2 = Realsense('./configs/basic_config/camera_rs_d435_c2.yaml')
    # test = ImageCollection(c1, c2)
    test = ImageCollection([c1])
    test.setObjectClass("plastic")
    test.run(saved_folder_path='./projects/ICRA2020/', auto_label=True, ref_img=['./d435i_static_100.avi'])


