# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/5
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""

import time
import cv2
import numpy as np


class BackgroundDetector(object):
    def __init__(self):
        self.fgmask = None
        self.fgbg = None

    def diffGround(self, groundImg, currrentImg, img_threshold=10, show_mask=False):
        """ generate mask from a background image"""
        # transfer to gray image
        groundImg_gray = cv2.cvtColor(groundImg, cv2.COLOR_BGR2GRAY)
        groundBlur = cv2.GaussianBlur(groundImg_gray, (3, 3), 1)
        groundBlur.dtype = 'int16'
        currrentImg_gray = cv2.cvtColor(currrentImg, cv2.COLOR_BGR2GRAY)
        currrentImgBlur = cv2.GaussianBlur(currrentImg_gray, (3, 3), 1)
        currrentImgBlur.dtype = 'int16'
        # subtraction
        dGrayBlur = abs(groundBlur-currrentImgBlur)
        dGrayBlur.dtype = 'uint8'
        dGrayMidBlur = cv2.medianBlur(dGrayBlur, 5)

        ret, thresh = cv2.threshold(dGrayMidBlur, img_threshold, 255, cv2.THRESH_BINARY)
        if show_mask:
            cv2.imshow('diff img', dGrayMidBlur)
            cv2.imshow('binary img from diff', thresh)
            cv2.waitKey()
        return thresh

    def filterColor(self, currrentImg, lower=np.array([10, 20, 0]), upper=np.array([60, 80, 40]), show_result=False):
        """ BGR channels"""
        mask = cv2.inRange(currrentImg, lower, upper)
        mask = cv2.bitwise_not(mask)
        if show_result:
            cv2.imshow('binary img from color', mask)
            cv2.waitKey()
        return mask

    def multiFrameLoader(self, ref_video='outpy.avi', ref_num=500, mog_threshold=20):
        """ load background video"""
        cap = cv2.VideoCapture(ref_video)
        self.fgbg = cv2.createBackgroundSubtractorMOG2(history=ref_num, detectShadows=True, varThreshold=mog_threshold)
        cnt = 0
        while (cnt < ref_num):
            ret, frame = cap.read()
            if not ret:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.fgmask = self.fgbg.apply(gray)
            # self.fgmask = cv2.medianBlur(self.fgmask, 5)

    def multiFrameFilter(self, color_img, show_mask=False):
        """ create Gaussian Mixture Model from multi images as background"""
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        mask = self.fgbg.apply(gray, self.fgmask, 0)
        # median filter
        mask = cv2.medianBlur(mask, 5)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.dilate(mask, kernel)
        mask = cv2.erode(mask, kernel, iterations=1)

        if show_mask:
            cv2.imshow('binary img from mog', mask)
            cv2.waitKey()
        return mask

    def grabCut_rect(self, color_img, rect=[200, 0, 900, 720]):
        """ rect = [col_min, row_min, col_max, row_max]"""
        mask = np.zeros(color_img.shape[:2], np.uint8)
        bgdModel = np.zeros((1, 65), np.float64)
        fgdModel = np.zeros((1, 65), np.float64)
        # rect = (200, 0, 900, 720)
        cv2.grabCut(color_img, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)
        mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
        return mask2

    def grabCut_mask(self, color_img, mask):
        # wherever it is marked white (sure foreground), change mask=1
        # wherever it is marked black (sure background), change mask=0
        mask[mask == 0] = 0
        mask[mask != 0] = 1
        bgdModel = np.zeros((1, 65), np.float64)
        fgdModel = np.zeros((1, 65), np.float64)
        mask, bgdModel, fgdModel = cv2.grabCut(color_img, mask, None, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_MASK)
        mask = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
        return mask

    def getConnectedDomain(self, binary_img, connectivity=4, region_area=1000, show_label=False):
        """ obtain connected domain"""
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_img, connectivity, cv2.CV_32S)
        # delete small regions
        label_index = []
        for i in range(num_labels):
            if stats[i][4] < region_area:
                labels[labels == i] = 0
            else:
                label_index.append(i)

        # Map component labels to hue val, 0-179 is the hue range in OpenCV
        if np.max(labels) == 0:
            label_hue = np.uint8(labels)
        else:
            label_hue = np.uint8(179*labels/np.max(labels))
        blank_ch = 255*np.ones_like(label_hue)
        labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])
        # Converting cvt to BGR
        labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)
        # set bg label to black
        labeled_img[label_hue == 0] = 0
        if show_label:
            cv2.imshow('color labels', labeled_img)
            cv2.waitKey()
        return labels, label_index, labeled_img

    def getConvexHull(self, img, show_convex=False):
        # convex hull
        result = cv2.findContours(np.uint8(img), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # contour: if hierarchy[0][i][3] == -1, it means there are contours inside
        # len(contours[i] is the num of the contour
        contours = []
        if len(result) == 2:
            contours = result[0]
            hierarchy = result[1]
        elif len(result) == 3:
            contours = result[1]
            hierarchy = result[2]

        hull = []
        for i in range(len(contours)):
            # creating convex hull object for each contour
            hull.append(cv2.convexHull(contours[i], False))

        if show_convex:
            # create an empty black image
            drawing = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
            # draw contours and hull points
            for i in range(len(contours)):
                color_contours = (0, 255, 0) # green - color for contours
                color = (255, 0, 0) # blue - color for convex hull
                # draw ith contour
                cv2.drawContours(drawing, contours, i, color_contours, 1, 8, hierarchy)
                # draw ith convex hull object
                cv2.drawContours(drawing, hull, i, color, 1, 8)
            cv2.imshow('convex', drawing)
            cv2.waitKey()
        return hull


if __name__ == '__main__':
    from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
    camera = Realsense('./configs/basic_config/camera_rs_d435_c1.yaml')
    frame = camera.get_frame()
    color = frame.color_image[0]
    time.sleep(1)
    frame = camera.get_frame()
    color = frame.color_image[0]
    save_ref_img = False
    if save_ref_img:
        cv2.imwrite('./projects/ICRA2020/ref_img.jpg', color)

    refImg = cv2.imread('./projects/ICRA2020/ref_img.jpg')
    cv2.imshow("a", refImg)
    cv2.waitKey()
    bd_test = BackgroundDetector()
    # generate mask
    # thresh = bd_test.diffGround(refImg, color, img_threshold=20, show_mask=True)
    thresh = bd_test.filterColor(color, show_result=True)
    # bd_test.multiFrameLoader(ref_video='d435qi_v40.avi', ref_num=500, mog_threshold=16)
    # thresh = bd_test.multiFrameFilter(color, show_mask=True)
    labels, labels_index, color_labels = bd_test.getConnectedDomain(thresh, show_label=True, region_area=2000)
    hul = bd_test.getConvexHull(labels, show_convex=True)











