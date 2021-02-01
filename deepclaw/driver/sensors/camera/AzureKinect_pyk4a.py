# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/9/26
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""

import sys, os, json, cv2, time
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
_current_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(_root_path)
sys.path.append(_current_path)
os.chdir(_root_path)
from deepclaw.driver.sensors.camera.camera_data import Frame

import pyk4a
from pyk4a_cfg import Config, PyK4A


class AzureKinect(object):
    def __init__(self, configuration_path = None, device=0):
        self.k4a = PyK4A(
            Config(
                color_resolution=pyk4a.ColorResolution.RES_720P,
                camera_fps=pyk4a.FPS.FPS_30,
                depth_mode=pyk4a.DepthMode.WFOV_2X2BINNED,
                synchronized_images_only=True,
            )
        )
        self.k4a.start()

    def get_frame(self):
        while True:
            capture = self.k4a.get_capture()
            if np.any(capture.depth) and np.any(capture.color):
                break
        # BGR
        img_color = capture.color
        color_img = []
        for i in range(len(img_color)):
            temp_col = []
            for j in range(len(img_color[0])):
                temp_col.append(img_color[i][j][0:3])
            color_img.append(temp_col)
        color_img = np.array(color_img)
        # depth image ,meter
        depth = capture.transformed_depth / 1000.0
        #
        pc = capture.depth_point_cloud.reshape((-1, 3))
        # pc = pc_from_depth(depth)
        return Frame([color_img], [depth], [pc], None)

    def get_intrinsics(self):
        return (602.365, 602.393, 636.734, 363.352, [None])

    def close(self):
        self.k4a.stop()


def pc_from_depth(dep_img):
    # - z = d / depth_scale
    # - x = (u - cx) * z / fx
    # - y = (v - cy) * z / fy
    width = 1280
    hight = 720
    fx = 602.365
    fy = 602.393
    cx = 636.734
    cy = 363.352
    pc = []
    for i in range(720):
        temp_pc = []
        for j in range(1280):
            z = dep_img[i][j]
            x = (j - cx) * z / fx
            y = (i - cy) * z / fy
            if z ==0:
                temp_pc.append([np.nan, np.nan, np.nan])
            else:
                temp_pc.append([x, y, z])
        pc.append(temp_pc)
    return np.array(pc).reshape((-1, 3))


if __name__ == "__main__":
    from matplotlib import pyplot as plt
    from mpl_toolkits import mplot3d
    camera = AzureKinect()
    time.sleep(1)
    frame = camera.get_frame()
    color_img = frame.color_image[0]
    depth_img = frame.depth_image[0]
    pcs = frame.point_cloud[0]
    print("=========show color image===========")
    plt.figure("color")
    plt.imshow(color_img[:,:,::-1])
    print("=========show depth image===========")
    plt.figure("depth")
    plt.imshow(depth_img)
    plt.show()
    print("=========show point clouds===========")
    # print(len(pcs))
    # color = color_img[:, :, ::-1].reshape((-1, 3))
    # fig = plt.figure("pcs")
    # ax = fig.add_subplot(111, projection="3d")
    # ax.scatter(pcs[:, 0], pcs[:, 1], pcs[:, 2], s=1, )
    # ax.set_xlabel("x")
    # ax.set_ylabel("y")
    # ax.set_zlabel("z")
    # ax.set_xlim(-1, 1)
    # ax.set_ylim(-1, 1)
    # ax.set_zlim(0.5, 2)
    # ax.view_init(elev=-90, azim=-90)
    # plt.show()



