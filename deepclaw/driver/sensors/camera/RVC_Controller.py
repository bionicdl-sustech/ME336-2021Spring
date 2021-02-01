# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/5/28
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: RVBUST RVC X 3D camera controller
"""
import PyRVC as RVC
import PyRVC as X1
import numpy as np
import os
import sys
current_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_path)
from camera_data import Frame

class RVC_Controller(object):
    # initialization from camera configuration file
    def __init__(self):
        self.x = []

    def open_device(self):
        X1.SystemInit()
        opt = X1.SystemListDeviceTypeEnum.USB
        ret, devices = X1.SystemListDevices(opt)
        self.x = X1.X1.Create(devices[0])
        self.x.Open()
        if self.x.IsOpen() == False or self.x.IsValid() == False:
            print("Open Camera False!")

    def get_frame(self):
        temp = self.x.Capture()
        if temp == False:
            print("Capture Data False")
        # 2D image
        image = self.x.GetImage()
        img = np.array(image.GetDataPtr())
        img = img.reshape(1200,1920).astype(np.uint8)
        # point clouds
        pts = self.x.GetPointMap()
        arr = np.array(pts.GetPointDataPtr())
        arr = arr.reshape(-1, 3)
        return Frame([img], None, [arr], None)

    def close_device(self):
        self.x.Close()
        X1.X1.Destroy(self.x)
        X1.SystemShutdown()


if __name__ == "__main__":
    rc = RVC_Controller()
    rc.open_device()
    import time
    t1 = time.time()
    cnt = 1
    for i in range(cnt):
        img = rc.get_frame()
    t2 = time.time()
    print("time: ",(t2-t1)/cnt)
    import matplotlib.pyplot as plt
    colorImg = img.color_image[0]
    plt.imshow(colorImg, cmap='gray')
    plt.show()
    pcs = img.point_cloud[0]
    # np.savetxt("./point_map.xyz",pcs)
    rc.close_device()