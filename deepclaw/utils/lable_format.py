# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/2/8
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: transfer mask format to other label format(yolo, coco .ect)
"""
import glob
import os
import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys
import json

def get_folder_list(target_path='./GraspingData/metal/'):
    folder_list = glob.glob(os.path.join(target_path, '*'))
    folder_list.sort()
    return folder_list


def get_mask_list(target_path='./20210125211711', camera_id='camera0', label_folder='labels_rembg'):
    label_path = os.path.join(target_path, camera_id, label_folder)
    mask_list = glob.glob(os.path.join(label_path, '*.pgm'))
    mask_list.sort()
    return mask_list


obj_c = {'glass': 0, 'paper': 1, 'metal': 2, 'plastic': 4}
is_debug = False
label_dict = {"version": [], "shapes": [], 'imagePath': [], "imageHeight": [], "imageWidth": []}
shape_dict = {"label": [], "mask_index": [], "mask_path": [], "position": [], "angle": []}


class LabelGenerator(object):
    """ transfer mask format to other label format(yolo, coco .ect)"""
    def __init__(self, input_path='./GraspingData/metal/', camera_id='camera0', src_label_folder='labels_rembg'):
        self.folder_list = glob.glob(os.path.join(input_path, '*'))
        self.folder_list.sort()

        self.cameraID = camera_id
        self.src_label = src_label_folder
        self.category = None

    def set_object_class(self, obj_class='paper'):
        self.category = obj_class

    def transfer2yolo(self):
        """ yolo label file is a txt file,
        each label information in it is 'class norm_center_x norm_center_y norm_width norm_height' """
        """ """
        yolo_label_path = 'labels_yolo'
        for i in self.folder_list:
            temp_mask_list = get_mask_list(i, camera_id=self.cameraID, label_folder=self.src_label)
            # create label folder
            output_label_path = os.path.join(i, self.cameraID, yolo_label_path)
            if not os.path.isdir(output_label_path):
                os.makedirs(output_label_path)

            with open(os.path.join(output_label_path, 'classes.txt'), 'a') as f:
                for sub_key in obj_c.keys():
                    f.write("%s\n" % sub_key)

            for temp_mask_img in temp_mask_list:
                mask = cv2.imread(temp_mask_img, cv2.IMREAD_ANYDEPTH)
                label_path = temp_mask_img.split('.')[0] + '.json'
                labels_file = temp_mask_img.split('/')[-1].split('.')[0] + '.txt'

                temp_mask = np.zeros(mask.shape, dtype='uint8')
                width = mask.shape[1]
                height = mask.shape[0]
                if not os.path.isfile(label_path):
                    if self.category is None:
                        print('Set object class!!')
                    temp_mask[np.where(mask != 0)] = 1
                    # stats: [col_left,row_top, width, height]
                    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(temp_mask, connectivity=4, ltype=cv2.CV_32S)
                    # plt.imshow(labels)
                    # plt.show()
                    if num_labels == 1:
                        # no object
                        continue
                    if is_debug:
                        for k in range(num_labels):
                            cv2.rectangle(temp_mask, (stats[k][0], stats[k][1]), (stats[k][0]+stats[k][2], stats[k][1]+stats[k][3]), (1,1,1), 2)
                            cv2.circle(temp_mask, (int(centroids[k][0]), int(centroids[k][1])), 2, (1, 1, 1), 2)
                        cv2.imshow('rect', temp_mask*255)
                        cv2.waitKey()
                    # object rectangle
                    for k in range(1, num_labels):
                        center_x = (stats[k][0] + stats[k][2]/2)/width
                        center_y = (stats[k][1] + stats[k][3]/2)/height
                        rect_width = stats[k][2]/width
                        rect_height = stats[k][3]/height
                        with open(os.path.join(output_label_path, labels_file), 'a') as f:
                            f.write("%d %.6f %.6f %.6f %.6f\n" % (obj_c[self.category], center_x, center_y, rect_width, rect_height))
                else:
                    with open(label_path) as f:
                        data = json.load(f)

                    for k in data['shapes']:
                        obj_mask = temp_mask.copy()
                        obj_mask[np.where(mask == k['mask_index'])] = 1
                        # plt.imshow(obj_mask)
                        # plt.show()
                        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(obj_mask, connectivity=4, ltype=cv2.CV_32S)
                        if num_labels == 1:
                            # no object
                            continue
                        if is_debug:
                            for m in range(1, num_labels):
                                cv2.rectangle(obj_mask, (stats[m][0], stats[m][1]), (stats[m][0]+stats[m][2], stats[m][1]+stats[m][3]), (1,1,1), 2)
                                cv2.circle(obj_mask, (int(centroids[m][0]), int(centroids[m][1])), 2, (1, 1, 1), 2)
                            cv2.imshow('rect', obj_mask*255)
                            cv2.waitKey()

                        # object rectangle
                        for m in range(1, num_labels):
                            center_x = (stats[m][0] + stats[m][2]/2)/width
                            center_y = (stats[m][1] + stats[m][3]/2)/height
                            rect_width = stats[m][2]/width
                            rect_height = stats[m][3]/height
                            with open(os.path.join(output_label_path, labels_file), 'a') as f:
                                f.write("%d %.6f %.6f %.6f %.6f\n" % (obj_c[k['label']], center_x, center_y, rect_width, rect_height))

    def transfer2coco(self):
        pass


if __name__ == "__main__":
    print('Label Format Test! \n')
    # object category
    folder_path = '/home/bionicdl-saber/Documents/GitHub/DeepClawDev/projects/ICRA2020/GraspingData/metal'
    ss = LabelGenerator(input_path=folder_path)
    ss.set_object_class('metal')
    ss.transfer2yolo()


