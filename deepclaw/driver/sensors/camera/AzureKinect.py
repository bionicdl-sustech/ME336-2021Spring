# Copyright (c) 2019 by Fang Wan. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
# This is the driver for Azure kinect and is dependent on Open3D==0.8.0
# Following the following steps before using this driver
# pip install open3d
# pip install open3d_azure_kinect_ubuntu1604_fix
#
# Copy 'https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules' into '/etc/udev/rules.d/'.

import sys, os, json, cv2, time
import numpy as np
import open3d as o3d

_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(_root_path)

from driver.sensors.camera.camera_data import Frame

# color resolution options
#{"K4A_COLOR_RESOLUTION_720P", K4A_COLOR_RESOLUTION_720P},
#{"K4A_COLOR_RESOLUTION_1080P", K4A_COLOR_RESOLUTION_1080P},
#{"K4A_COLOR_RESOLUTION_1440P", K4A_COLOR_RESOLUTION_1440P},
#{"K4A_COLOR_RESOLUTION_1536P", K4A_COLOR_RESOLUTION_1536P},
#{"K4A_COLOR_RESOLUTION_2160P", K4A_COLOR_RESOLUTION_2160P},
#{"K4A_COLOR_RESOLUTION_3072P", K4A_COLOR_RESOLUTION_3072P},


class AzureKinect(object):
    def __init__(self, configuration_path = None, device=0):
        if configuration_path is not None:
            config = o3d.io.read_azure_kinect_sensor_config(configuration_path)
        else:
            config = o3d.io.AzureKinectSensorConfig()
        self.align_depth_to_color = True

        self.sensor = o3d.io.AzureKinectSensor(config)

        if device < 0 or device > 255:
            print('Unsupported device id, fall back to 0')
            device = 0
        if not self.sensor.connect(device):
            raise RuntimeError('Failed to connect to sensor')

        # config_dict = json.loads(open(configuration_path).read())
        # self.color_resolution = config_dict['color_resolution']
        self.color_resolution = 'K4A_COLOR_RESOLUTION_720P'
        if self.color_resolution == 'K4A_COLOR_RESOLUTION_720P':
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic(1280, 720, 602.365, 602.393, 636.734, 363.352)
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_1080P':
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic(1920, 1080, 903.547, 903.589, 955.35, 545.277)
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_1440P':
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic(2560, 1440, 1204.73, 1204.79, 1273.97, 727.203)
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_1536P':
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic(2048, 1536, 963.783, 963.829, 1019.07, 773.662)
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_2160P':
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic(3840, 2160, 1807.09, 1807.18, 1911.2, 1091.05)
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_3072P':
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic(4096, 3072, 1927.57, 1927.66, 2038.65, 1547.82)
        else:
            self.intrinsics = None

    def get_frame(self):
        rgbd = self.sensor.capture_frame(self.align_depth_to_color)
        # the API returned color image is in RGB order
        # convert to BGR
        color_image = np.asarray(rgbd.color)[:,:,::-1]
        depth_image = np.asarray(rgbd.depth)/1000.0 # original unit is mm, converting to m

        # compute point cloud from depth image
        pointcloud = o3d.geometry.PointCloud.create_from_depth_image(rgbd.depth, self.intrinsics, depth_scale=1000, project_valid_depth_only=False)
        pointcloud_xyz =  np.array(pointcloud.points)

        return Frame([color_image], [depth_image], [pointcloud_xyz], None)

    def get_intrinsics(self):
        if self.color_resolution == 'K4A_COLOR_RESOLUTION_720P':
            return (602.365, 602.393, 636.734, 363.352, [None])
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_1080P':
            return (903.547, 903.589, 955.35, 545.277, [None])
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_1440P':
            return (1204.73, 1204.79, 1273.97, 727.203, [None])
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_1536P':
            return (963.783, 963.829, 1019.07, 773.662, [None])
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_2160P':
            return (1807.09, 1807.18, 1911.2, 1091.05, [None])
        elif self.color_resolution == 'K4A_COLOR_RESOLUTION_3072P':
            return (1927.57, 1927.66, 2038.65, 1547.82, [None])
        else:
            self.intrinsics = None

if __name__ == '__main__':
    # camera  = AzureKinect('../../../../configs/robcell-ur10e-hande-kinectazure/AzureKinect.json')
    camera  = AzureKinect()
    time.sleep(1)
    frame = camera.get_frame()
    cv2.imshow("test", frame.color_image[0])
    cv2.waitKey()
    cv2.destroyAllWindows()
    print(camera.get_intrinsics())
    print("Saved test image to test_AzureKinect.png")
