# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/12/31
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: transfer images to video
"""

import sys
import os
# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)

import cv2
import numpy as np
from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
import time


class RecordImage(object):
    def __init__(self, camera_serve):
        self.camera = camera_serve
        time.sleep(0.5)

    def saveVideo(self, file_name='./outpy.avi', video_duration=8.0):
        """ save as video"""
        frame = self.camera.get_frame()
        color = frame.color_image[0]
        height, width, channels = color.shape
        out = cv2.VideoWriter(file_name, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20, (width, height))
        t1 = time.time()
        while True:
            frame = self.camera.get_frame()
            color = frame.color_image[0]

            out.write(color)
            cv2.imshow('frame', color)
            t2 = time.time()
            if t2-t1 > video_duration or (cv2.waitKey(1) & 0xFF == ord('q')):
                break

        out.release()
        cv2.destroyAllWindows()

    def saveImage(self, file_name='./out.jpg'):
        """ save as single image"""
        frame = self.camera.get_frame()
        color = frame.color_image[0]
        cv2.imwrite(file_name, color)
        cv2.imshow('frame', color)
        cv2.waitKey()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    camera = Realsense('./configs/basic_config/camera_rs_d435_c1.yaml')
    record_s = RecordImage(camera)
    # save video
    record_s.saveVideo(file_name='d435i_static_100.avi', video_duration=10)
    # save image
    record_s.saveImage(file_name='bg_ing.jpg')




