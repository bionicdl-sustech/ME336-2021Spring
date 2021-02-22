# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/8/26
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""
import numpy as np
import cv2
import yaml


class Calibration2D(object):
    def __init__(self, cfg_path):
        __cfg = yaml.load(open(cfg_path, 'r'), Loader=yaml.FullLoader)
        # [x, y]
        xy1 = __cfg['xy1']
        xy2 = __cfg['xy2']
        xy3 = __cfg['xy3']
        xy4 = __cfg['xy4']
        # [col, row]
        uv1 = __cfg['uv1']
        uv2 = __cfg['uv2']
        uv3 = __cfg['uv3']
        uv4 = __cfg['uv4']

        # perspective transformation
        pts1 = np.float32([uv1, uv2, uv3, uv4])
        pts2 = np.float32([xy1, xy2, xy3, xy4])
        self.image2baseMatrix = cv2.getPerspectiveTransform(pts1, pts2)

    def cvt(self, u, v):
        """image to robot base"""
        pick_point = [u, v]
        grasp_point = np.array([[pick_point]], dtype=np.float32)
        gp_base = cv2.perspectiveTransform(grasp_point, self.image2baseMatrix)
        x = gp_base[0][0][0]
        y = gp_base[0][0][1]
        return x, y

    def robot2img(self, x, y):
        """robot base to image"""
        pick_point = [x, y]
        grasp_point = np.array([[pick_point]], dtype=np.float32)
        temp = cv2.perspectiveTransform(grasp_point, np.linalg.inv(self.image2baseMatrix))
        return temp[0][0][0], temp[0][0][1]


if __name__ == '__main__':
    path = '../../../configs/ICRA2020-ur5-azure-rg6/cali2D.yaml'
    hand_eye = Calibration2D(path)
    print(hand_eye.cvt(471, 554))
    print(hand_eye.image2baseMatrix)
    print(hand_eye.robot2img(-0.22718306, -0.96182245))


