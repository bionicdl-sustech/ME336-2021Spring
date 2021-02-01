# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/15
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

import cv2
import os
import glob
import numpy as np
import matplotlib.pyplot as plt


check_flag = 0
if check_flag:
    data_path = "/home/bionicdl-saber/Documents/GitHub/DeepClawDev/projects/ICRA2020/GraspingData/20210119232350/camera0"
    color_img_path = os.path.join(data_path, "images", "color")
    label_path = os.path.join(data_path, "labels")

    file_list = glob.glob(label_path + '/*.jpg')
    file_list.sort()

    for i in range(len(file_list)):
        # print(file_list[i].split('/')[-1])
        img_name = file_list[i].split('/')[-1]
        ori_img = cv2.imread(color_img_path + "/" + img_name)
        mask_img = cv2.imread(label_path + "/" + img_name)
        overlapping = cv2.addWeighted(ori_img, 0.5, mask_img, 0.5, 0)
        cv2.imshow('result', overlapping)
        cv2.waitKey()
        # cv2.destroyAllWindows()
    exit()


key_press_test = 0
if key_press_test:
    cv2.namedWindow('image')
    while 1:
        key_num = cv2.waitKey(1)
        if key_num != -1:
            print(key_num)
        if key_num == 113:  # press 'q' for exit
            break
        elif key_num == 32:  # press space for pause
            print('Pause Program!')
            print("Press Enter to continue...")
            cnt = 0
            while 1:
                cnt += 1
                print(cnt)
                recover_key = cv2.waitKey(100)
                if recover_key == 13:  # press enter for recovering
                    break
    exit()

# roi mask
mask_test = 0
if mask_test:
    col = 1280
    row = 720
    region_parameter = [200, 1100, 0, 720]  # col_min, col_max, row_min, row_max


    region_mask = np.zeros((720, 1280), dtype=np.uint8)
    image_1 = np.ones((720, 1280), dtype=np.uint8) * 255

    region_mask[region_parameter[2]:region_parameter[3], region_parameter[0]:region_parameter[1]] = image_1[region_parameter[2]:region_parameter[3], region_parameter[0]:region_parameter[1]]

    ori_img = cv2.imread('/home/bionicdl-saber/Documents/GitHub/DeepClawDev/projects/ICRA2020/GraspingData/20210119104232/camera0/images/color/00000007.jpg')
    print(ori_img.shape[0])
    xxx = cv2.cvtColor(ori_img, cv2.COLOR_BGR2GRAY)
    xxx = np.multiply(xxx, region_mask)
    cv2.imshow('gray', xxx)
    cv2.waitKey()

float_img_write = 0
if float_img_write:
    # depth_path = '/home/bionicdl-saber/Documents/GitHub/DeepClawDev/projects/ICRA2020/GraspingData/20210119232350/camera0/images/depth/00000150.npy'
    # depth_img = np.load(depth_path)
    # depth_img = (depth_img*1000).astype(np.uint16)
    # print(type(depth_img))
    # plt.imshow(depth_img)
    # plt.show()

    tt = '/home/bionicdl-saber/Documents/GitHub/DeepClawDev/projects/ICRA2020/GraspingData/paper/20210125182611/camera0/labels/00000000.pgm'

    ss = cv2.imread(tt, cv2.IMREAD_ANYDEPTH)
    # ss = cv2.imread(tt)
    print(ss.shape)
    plt.imshow(ss)
    plt.show()


grabCut_flag = 1
if grabCut_flag:
    tt = ('/home/bionicdl-saber/Music/xx.jpg')
    img = cv2.imread(tt)
    mask = np.zeros(img.shape[:2], np.uint8)
    bgdModel = np.zeros((1, 65), np.float64)
    fgdModel = np.zeros((1, 65), np.float64)
    rect = [200, 0, 900, 720]
    cv2.grabCut(img, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
    # plt.imshow(mask)
    # plt.colorbar()
    # plt.show()
    mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
    # plt.imshow(mask2)
    # plt.colorbar()
    # plt.show()
    img = img*mask2[:, :, np.newaxis]
    # plt.imshow(img)
    # plt.colorbar()
    # plt.show()

    xx = ('/home/bionicdl-saber/Documents/GitHub/DeepClawDev/projects/ICRA2020/'
          'GraspingData/plastic/20210125212810/camera0/labels/00000000.pgm')
    # newmask is the mask image I manually labelled
    newmask = cv2.imread(xx, cv2.IMREAD_ANYDEPTH)
    # wherever it is marked white (sure foreground), change mask=1
    # wherever it is marked black (sure background), change mask=0
    mask[newmask == 0] = 0
    mask[newmask == 12] = 1
    mask, bgdModel, fgdModel = cv2.grabCut(img,mask,None,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_MASK)
    mask = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    # img = img*mask[:, :, np.newaxis]
    plt.imshow(mask)
    plt.colorbar()
    plt.show()


from rembg.bg import remove
import numpy as np
import io
from PIL import Image
import filetype
from tqdm import tqdm
#
# input_path = ('/home/bionicdl-saber/Music/xx.jpg')
# output_path = 'out.png'
# xx = '/home/bionicdl-saber/Music/xx.png'
#
# input_path = ['/home/bionicdl-saber/Music/xx.jpg']
# full_paths = [os.path.abspath(mm) for mm in input_path]
# files = set()
# files.add(full_paths[0])
# print(files)
#
# r = lambda i: i.buffer.read() if hasattr(i, "buffer") else i.read()
#
# for fi in tqdm(files):
#     fi_type = filetype.guess(fi)
#     print('fi_type', fi_type)
#     print(fi_type.mime.find("image"))
#     if fi_type is None:
#         continue
#     elif fi_type.mime.find("image") < 0:
#         continue
#
#     with open(fi, "rb") as input:
#         if hasattr(input, "buffer"):
#             ss = input.buffer.read()
#         else:
#             ss = input.read()
#         # result = remove(r(input))
#         result = remove(ss)
#         print('type result: ', type(result))
#         img = Image.open(io.BytesIO(result)).convert("RGB")
#         plt.imshow(np.array(img))
#         plt.show()
#         print('xxxxxxxxxxxxxxx')


def rembg_detector(file_path):
    with open(file_path, "rb") as input:
        if hasattr(input, "buffer"):
            ss = input.buffer.read()
        else:
            ss = input.read()
    result = remove(ss)
    print('type result: ', type(result))
    img = Image.open(io.BytesIO(result)).convert("RGBA")
    return np.array(img)


rembg_flag = 0
if rembg_flag:
    data_path = ("/home/bionicdl-saber/Documents/GitHub/DeepClawDev/"
                 "projects/ICRA2020/GraspingData/速度测试/20210115162934/camera0")
    color_img_path = os.path.join(data_path, "images", "color")
    file_list = glob.glob(color_img_path + '/*.jpg')
    file_list.sort()
    print(file_list)

    for i in range(len(file_list)):
        # print(file_list[i].split('/')[-1])
        img_name = file_list[i].split('/')[-1]
        result_mask = rembg_detector(color_img_path + "/" + img_name)
        # ori_img = cv2.imread(color_img_path + "/" + img_name)
        plt.imshow(result_mask)
        plt.pause(0.2)
        plt.clf()


        # cv2.imshow('result', result_mask)
        # cv2.waitKey(10)
        # cv2.destroyAllWindows()

















