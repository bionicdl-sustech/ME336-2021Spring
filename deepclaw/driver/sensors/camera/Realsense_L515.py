# Copyright (c) 2020 by BionicLab. All Rights Reserved.
# !/usr/bin/python
# -*- coding:utf-8 -*-
"""
@File: Realsense
@Author: Haokun Wang
@ Modified: Liu Xiaobo
@Date: 2020/4/3 15:47
@Description:
The pyrealsense2 API can be found in https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html.
@ update L515 support, 20201210
"""
import pyrealsense2 as rs
import yaml
import numpy as np
import time
from deepclaw.driver.sensors.camera.camera_data import Frame


class Realsense(object):
    # initialization from camera configuration file
    def __init__(self, camera_configuration_file_path):
        self._cfg = yaml.load(open(camera_configuration_file_path, 'r'), Loader=yaml.FullLoader)
        self.serial_number = self._cfg["serial_number"]

        self.points = rs.points()
        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.serial_number != '' and self.serial_number != 'None':
            print('Config for realsense %s' % self.serial_number)
            config.enable_device(self.serial_number)

        # the resolution of color image can be higher than depth image
        if self._cfg["name"] == 'L515':
            config.enable_stream(rs.stream.infrared, self._cfg['depth_infrared']['width'], self._cfg['depth_infrared']['height'], rs.format.y8, self._cfg['depth_infrared']['fps'])
            config.enable_stream(rs.stream.depth, self._cfg['depth_infrared']['width'], self._cfg['depth_infrared']['height'], rs.format.z16, self._cfg['depth_infrared']['fps'])
            config.enable_stream(rs.stream.color, self._cfg['color']['width'], self._cfg['color']['height'], rs.format.bgr8, self._cfg['color']['fps'])
        else:
            config.enable_stream(rs.stream.infrared, 1, self._cfg['depth_infrared']['width'], self._cfg['depth_infrared']['height'], rs.format.y8, self._cfg['depth_infrared']['fps'])
            config.enable_stream(rs.stream.infrared, 2, self._cfg['depth_infrared']['width'], self._cfg['depth_infrared']['height'], rs.format.y8, self._cfg['depth_infrared']['fps'])
            config.enable_stream(rs.stream.depth, self._cfg['depth_infrared']['width'], self._cfg['depth_infrared']['height'], rs.format.z16, self._cfg['depth_infrared']['fps'])
            config.enable_stream(rs.stream.color, self._cfg['color']['width'], self._cfg['color']['height'], rs.format.bgr8, self._cfg['color']['fps'])

        self.profile = self.pipeline.start(config)

        if "exposure" in self._cfg['color'].keys() and self._cfg['color']['exposure'] != 'None':
            # set camera parameters
            for i in range(len(self.profile.get_device().sensors)):
                if self.profile.get_device().sensors[i].is_color_sensor():
                    # set color camera
                    color_sensor = self.profile.get_device().sensors[i]
                    color_sensor.set_option(rs.option.enable_auto_exposure, 0)
                    color_sensor.set_option(rs.option.exposure, self._cfg['color']['exposure'])
                    # color_sensor.set_option(rs.option.gain, 100)
                    print("camera exposure: ", color_sensor.get_option(rs.option.exposure)*0.1, 'ms')

        '''
        The align class used in librealsense demos maps between depth and some other stream and vice versa. 
        Realsense do not offer other forms of stream alignments.
        https://github.com/IntelRealSense/librealsense/issues/1556
        '''
        self.basic_img = self._cfg["align_image"]
        if self._cfg["align_image"] == 'color':
            align_to = rs.stream.color
        elif self._cfg["align_image"] == 'depth':
            align_to = rs.stream.depth
        else:
            print("Error align image!")
            print("Align to color!")
            align_to = rs.stream.color

        self.align = rs.align(align_to)

        self.scale = self.get_depth_scale()

    """
    @Description: get images from a camera
    @parameters[in]: None
    @return:
        color_image: color image
        depth_image: depth image, unit:m
        point_cloud: point cloud ,unit:m
        infrared_L, infrared_R: left infrared image and right infrared image
        infrared: infrared image (only one image)
    """
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        if self._cfg["name"] == 'L515':
            aligned_ir_frame = aligned_frames.get_infrared_frame()
        else:
            aligned_irL_frame = aligned_frames.get_infrared_frame(1)
            aligned_irR_frame = aligned_frames.get_infrared_frame(2)

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_image = depth_image*self.scale
        color_image = np.asanyarray(aligned_color_frame.get_data())
        if self._cfg["name"] == 'L515':
            infrared = np.asanyarray(aligned_ir_frame.get_data())
        else:
            infrared_L = np.asanyarray(aligned_irL_frame.get_data())
            infrared_R = np.asanyarray(aligned_irR_frame.get_data())

        pc = rs.pointcloud()
        point_cloud = pc.calculate(aligned_depth_frame)
        # point_cloud = None
        if self._cfg["name"] == 'L515':
            return Frame([color_image], [depth_image], [point_cloud], [infrared])
        else:
            return Frame([color_image], [depth_image], [point_cloud], [infrared_L, infrared_R])

    """
    @Description: get intrinsics attributes of a camera
    @parameters[in]: None
    @return:
        intrinsics.fx: focal length of the image in width(columns)
        intrinsics.fy: focal length of the image in height(rows)
        intrinsics.ppx: the pixel coordinates of the principal point (center of projection) in width
        intrinsics.ppy: the pixel coordinates of the principal point (center of projection) in height
    """
    def get_intrinsics(self):
        color_stream = self.profile.get_stream(rs.stream.color)
        depth_stream = self.profile.get_stream(rs.stream.depth)
        infrared_stream = self.profile.get_stream(rs.stream.infrared)

        intrinsics_color = color_stream.as_video_stream_profile().get_intrinsics()
        intrinsics_depth = depth_stream.as_video_stream_profile().get_intrinsics()
        intrinsics_infrared = infrared_stream.as_video_stream_profile().get_intrinsics()
        # '=====intrinsics=========='
        # (intrinsics_color.fx, intrinsics_color.fy, intrinsics_color.ppx, intrinsics_color.ppy, intrinsics_color.distCoeffs)
        return (intrinsics_color, intrinsics_depth, intrinsics_infrared)

    def get_extrinsic_color2depth(self):
        color_stream = self.profile.get_stream(rs.stream.color)
        depth_stream = self.profile.get_stream(rs.stream.depth)
        infrared_stream = self.profile.get_stream(rs.stream.infrared)
        color2depth = color_stream.get_extrinsics_to(depth_stream)
        return color2depth

    def get_device(self):
        return self.profile.get_device()

    def get_depth_scale(self):
        depth_sensor = self.profile.get_device().first_depth_sensor()
        return depth_sensor.get_depth_scale()

    def get_serial_number(self):
        device = self.get_device()
        return str(device.get_info(rs.camera_info.serial_number))


if __name__ == '__main__':
    import os
    import sys
    _root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
    sys.path.append(_root_path)
    os.chdir(_root_path)
    print('work_dir: ', _root_path)

    import matplotlib.pyplot as plt
    camera = Realsense('./configs/basic_config/camera_rs.yaml')
    # x = camera.get_frame()
    time.sleep(0.1)
    x = camera.get_frame()
    color = x.color_image[0]
    infrared_image = x.infrared_image[0]
    dd = x.depth_image[0]
    print('color shape: ', color.shape)
    print('infrared shape: ', infrared_image.shape)
    print('depth shape: ', dd.shape)

    plt.imshow(color)
    plt.imshow(x.infrared_image[0])
    plt.show()
    depth = x.depth_image
    pc = x.point_cloud
    # print(depth[0])
    intrinsics = camera.get_intrinsics()
    print('intrinsics: ', intrinsics)
    print(type(intrinsics), len(intrinsics))
    print(intrinsics[0].ppx)
    print('depth scale: ', camera.get_depth_scale())
    print('serial number: ', camera.get_serial_number())
    # show point cloud
    show_pc = False
    if show_pc:
        vtx = np.asarray(pc[0].get_vertices())
        vtx = np.array(vtx.tolist())
        cc = x.color_image[0]
        # BGR to RGB
        cc = cc[:, :, ::-1]
        cc = cc.reshape(-1, 3)

        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vtx)
        pcd.colors = o3d.utility.Vector3dVector(cc.astype(np.float32)/255)

        bbox = o3d.geometry.AxisAlignedBoundingBox()
        bbox.min_bound = [-1, -1, 0.3]
        bbox.max_bound = [1, 0.6, 1.2]
        pcd = pcd.crop(bbox)
        o3d.visualization.draw_geometries([pcd],window_name='model', point_show_normal=True)
