# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8


# class CameraController(object):
#     def __init__(self):
#         pass
#
#     def get_frame(self):
#         raise NotImplementedError(' getImage method does not implement. ')
#
#     def get_intrinsics(self):
#         raise NotImplementedError(' getIntrinsics method does not implement. ')


class Frame(object):
    def __init__(self, color_image, depth_image, point_cloud, infrared_image):
        self.color_image = color_image
        self.depth_image = depth_image
        self.point_cloud = point_cloud
        self.infrared_image = infrared_image

    def get_serial_number(self, serial_number):
        self.serial_number = serial_number
