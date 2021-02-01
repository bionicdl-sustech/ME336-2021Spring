# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/12/31
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
import numpy as np
import skimage
from skimage import measure, color
from skimage.measure import find_contours, approximate_polygon, subdivide_polygon
from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
import matplotlib.pyplot as plt
# while(cap.isOpened()):
#     ret, frame = cap.read()
#     if frame is None:
#         break
#     # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#
#     cv2.imshow('frame', frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#
# cap.release()
# cv2.destroyAllWindows()


cap = cv2.VideoCapture('outpy.avi')
fgbg = cv2.createBackgroundSubtractorMOG2(history=100, detectShadows=True, varThreshold=100)
cnt = 0
fgmask = None
while(cnt < 200):
    ret, frame = cap.read()
    if not ret:
        print(cnt)
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    fgmask = fgbg.apply(gray)
    fgmask = cv2.medianBlur(fgmask, 5)

    # cv2.imshow('frame', fgmask)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    cnt += 1

camera = Realsense('./configs/basic_config/camera_rs_d435.yaml')
while True:
    frame = camera.get_frame()
    color = frame.color_image[0]
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    # fftest = fgbg.apply(gray)
    xx = fgbg.apply(gray, fgmask, 0)
    # 膨胀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
    xx = cv2.dilate(xx, kernel)
    # 中值滤波
    xx = cv2.medianBlur(xx, 5)

    # 连通域
    alg_test = 'skimage'
    if alg_test == 'opencv':
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(xx, connectivity=8)
        print('num_labels: ', num_labels)
        print("stats: ", stats)
        print("centroids: ", centroids)

        # Map component labels to hue val, 0-179 is the hue range in OpenCV
        label_hue = np.uint8(179*labels/np.max(labels))
        blank_ch = 255*np.ones_like(label_hue)
        labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

        # Converting cvt to BGR
        labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

        # set bg label to black
        labeled_img[label_hue==0] = 0
        cv2.imshow('cc', labeled_img)
    elif alg_test == 'skimage':
        labels = skimage.measure.label(xx, connectivity=2)
        dd = skimage.measure.regionprops(labels)
        print(len(dd))
        dst = skimage.color.label2rgb(labels)  #根据不同的标记显示不同的颜色
        print('regions number:', labels.max()+1)  #显示连通区域块数(从0开始标记)
        fig, axi2 = plt.subplots(nrows=1, ncols=1)

        for region in dd:
            # take regions with large enough areas
            if region.area >= 1000:
                # draw rectangle around segmented coins
                minr, minc, maxr, maxc = region.bbox
                rect = cv2.rectangle(dst, (minc, minr),(maxc, maxr),(0,0,255),2)
                # print('xxx:', region.coords)
                new_xx = subdivide_polygon(region.coords, degree=5, preserve_ends=True)
                coords = approximate_polygon(new_xx, tolerance=40)
                print('yyy:', coords)

                axi2.plot(coords[:, 1], coords[:, 0], '-g', linewidth=2)
        cv2.imshow('cc', dst)
        plt.show()

        # approximate / simplify coordinates of the
        # for contour in find_contours(xx, 0):
        #     coords = approximate_polygon(contour, tolerance=2.5)
        #     coords2 = approximate_polygon(contour, tolerance=39.5)
        #     print("Number of coordinates:", len(contour), len(coords), len(coords2))

    cv2.imshow('xx', xx)
    # cv2.imshow('cc', color)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

