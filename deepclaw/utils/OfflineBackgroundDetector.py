# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/27
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: Detect objects, generate object mask for collected images
"""
import sys
import os
# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)


import os
import glob
from rembg.bg import remove
import numpy as np
import io
import json
from PIL import Image
import matplotlib.pyplot as plt
import cv2
import math
from sklearn.decomposition import PCA
from deepclaw.utils.ForegroundDetector import BackgroundDetector


def rembg_detector(file_path):
    """ Rembg is a tool to remove images background. https://github.com/danielgatis/rembg"""
    with open(file_path, "rb") as input:
        if hasattr(input, "buffer"):
            ss = input.buffer.read()
        else:
            ss = input.read()
    result = remove(ss)
    img = Image.open(io.BytesIO(result)).convert("RGBA")
    img = np.array(img)
    # alpha value
    mask_binary = img[:, :, 3]
    mask_binary[mask_binary > 10] = 255
    mask_binary[mask_binary <= 10] = 0
    # plt.imshow(img)
    # plt.show()
    return mask_binary


def get_file_path(folder_path, image_type='jpg'):
    """ find all files in a folder with same suffix"""
    file_path_list = glob.glob(folder_path + '/*.%s' % image_type)
    return file_path_list


def save_label(file_list, label_path, region_parameter, obj_class=None, detect_algo='rembg', ref_frame='./ref.avi'):
    """ auto label the exited images"""
    label_dict = {"version": [], "shapes": [], 'imagePath': [], "imageHeight": [], "imageWidth": []}
    shape_dict = {"label": [], "mask_index": [], "mask_path": [], "position": [], "angle": []}

    mask_detector = BackgroundDetector()
    for i in range(len(file_list)):
        color_img = cv2.imread(file_list[i])
        # generate object mask
        bi_mask = None
        if detect_algo == 'rembg':
            bi_mask = rembg_detector(file_list[i])
        elif detect_algo == 'colorFilter':
            lower = np.array([10, 20, 0])
            upper = np.array([60, 80, 40])
            bi_mask = mask_detector.filterColor(color_img, lower=lower, upper=upper)
        elif detect_algo == 'mog2':
            "for mog2, the ref_frame is a video, xx.avi for example"
            mask_detector.multiFrameLoader(ref_frame, ref_num=1000, mog_threshold=80)
            bi_mask = mask_detector.multiFrameFilter(color_img)
        elif detect_algo == 'diff':
            "for diff, the ref_frame is a picture, xx.jpg for example"
            temp_color = cv2.imread(ref_frame)
            bi_mask = mask_detector.diffGround(temp_color, color_img)
        else:
            print("Error detector algorithm set!")

        if bi_mask is None:
            continue

        img_index = file_list[i].split('/')[-1].split('.')[0]  # get file name without suffix
        # ROI, filter mask with roi
        # region_parameter = [200, 1080, 0, 720]
        height_row = color_img.shape[0]
        width_col = color_img.shape[1]
        region_mask = np.zeros((height_row, width_col), dtype=np.uint8)
        image_1 = np.ones((height_row, width_col), dtype=np.uint8)
        region_col_min = region_parameter[0]
        region_col_max = region_parameter[1]
        region_row_min = region_parameter[2]
        region_row_max = region_parameter[3]
        region_mask[region_row_min:region_row_max, region_col_min:region_col_max] = image_1[region_row_min:region_row_max, region_col_min:region_col_max]
        bi_mask = np.multiply(bi_mask, region_mask)

        # label each instance
        # mask_detector = BackgroundDetector()
        labels, labels_index, color_labels = mask_detector.getConnectedDomain(bi_mask, show_label=False, region_area=2000)
        # show image and mask
        overlapping = cv2.addWeighted(color_img, 0.6, color_labels, 0.4, 0)
        cv2.imshow('result', overlapping)
        key_num = cv2.waitKey(100)
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

        if len(labels_index) < 2:  # there is always background label
            continue
        # save mask label
        seg_mask_name = img_index + '.pgm'
        color_mask_name = img_index + '.jpg'
        cv2.imwrite(label_path + '/' + color_mask_name, overlapping)
        cv2.imwrite(label_path + '/' + seg_mask_name, labels.astype(np.uint16))
        label_dict["version"] = '4.5.6'
        label_dict["imagePath"] = '../images/color/' + file_list[i].split('/')[-1]
        label_dict["imageHeight"] = color_img.shape[0]
        label_dict["imageWidth"] = color_img.shape[1]

        # generator object grasping data with PCA
        pick_temp = []
        for k in range(len(labels_index)):
            if labels_index[k] == 0:
                continue
            else:
                shape_dict["label"] = obj_class
                shape_dict["mask_index"] = labels_index[k]
                shape_dict["mask_path"] = seg_mask_name

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

                # shape_dict["label"] = obj_class
                shape_dict["position"] = pca.mean_.tolist()  # col, row
                shape_dict["angle"] = theta
                pick_temp.append(shape_dict.copy())

        label_dict["shapes"] = pick_temp
        # save label file
        labels_name = img_index + '.json'
        with open(label_path + '/' + labels_name, 'w') as f:
            json.dump(label_dict, f, indent=4)


if __name__ == "__main__":
    # object category
    obj_class = "paper"
    folder_path = '/home/bionicdl-saber/Documents/GitHub/DeepClawDev/projects/ICRA2020/GraspingData/metal/*'
    """
    the image folder tree is:
    folder_path
    ├── 20210125184650
    │   └── camera0
    │       ├── images
    │       │   ├── color
    |       |   |   ├── 00000000.jpg
    |       |   |   ├── 00000001.jpg
    │       │   ├── depth
    |       |   |   ├── 00000000.pgm
    |       |   |   ├── 00000001.pgm
    │       │   └── infrared
    |       |       ├── 00000000_L.jpg
    |       |       ├── 00000000_R_.jpg
    |       │       ├── 00000001_L.jpg
    |       │       ├── 00000001_R_.jpg
    |       └── labels_rembg
    |           ├── 00000002.jpg
    |           ├── 00000002.json
    |           ├── 00000002.pgm
    |           ├── 00000008.jpg
    |           ├── 00000008.json
    |           ├── 00000008.pgm
    the folder named "images" is collected images folder, "labels_rembg" folder is generated label folder and 
    the folder name can be set with the variable 'label_folder_name'.
    In the label folder, the *.jpg file is color image for checking mask, 
    *.pgm file is depth image stored depth information with np.uint16 type 
    (using >> cv2.imread(file_path, cv2.IMREAD_ANYDEPTH) to load depth), 
    and *.json file is label file with structure:
        {
        "version": "4.5.6",
        "shapes": [
            {
                "label": "paper",
                "mask_index": 1,
                "mask_path": "00000000.npy",
                "position": [
                    644.0554294328516,
                    366.55604745902633
                ],
                "angle": -0.010259211844891744
            },
        ],
        "imagePath": "../images/color/00000000.jpg",
        "imageHeight": 720,
        "imageWidth": 1280
        }
    """
    folder_list = glob.glob(folder_path)
    folder_list.sort()
    region_parameter = [200, 1120, 0, 720]
    camera_sel = "camera0"
    label_folder_name = "labels_rembg"
    for i in folder_list:
        print('folder name: ', i)
        color_img_path = os.path.join(i, camera_sel, "images", "color")
        label_path = os.path.join(os.path.join(i, camera_sel), label_folder_name)
        # create label folder
        if not os.path.isdir(label_path):
            os.makedirs(label_path)
        file_list = get_file_path(color_img_path, 'jpg')
        file_list.sort()
        save_label(file_list, label_path, region_parameter, obj_class=obj_class)
