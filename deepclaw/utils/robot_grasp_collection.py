# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/12/28
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""

from deepclaw.driver.sensors.camera.AzureKinect_pyk4a import AzureKinect
from deepclaw.driver.arms.URController_rtde import URController
from deepclaw.modules.calibration.Calibration2D import Calibration2D
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import math
import time
import os
import json

def VisLabel(img1,img2,compare_space=[630,830,50,100]):
    return 1


def AutoGraspLabel(robot_serve, gripper_serve, camera_serve, ref_img, pick_xyzrt):
    middle_point = np.array([-0.058, -0.573, 0.54, 2.2030, -2.2068, 0.0156])
    vis_label_pos = np.array([0.429, -0.436, 0.31, 1.5711, -1.8591, 0.6345])
    label_up = vis_label_pos.copy()
    label_up[2] = label_up[2] + 0.1
    # go above the pick pos
    up_pos = pick_xyzrt.copy()
    up_pos[2] = up_pos[2] + 0.05
    robot_serve.move_L(up_pos, 1.5, 1.5)
    # go to pick
    robot_serve.move_L(pick_xyzrt, 1.5, 1.5)
    gripper_serve.set_tool_out(0, True)
    time.sleep(2)
    # go up
    robot_serve.move_L(up_pos, 1.5, 1.5)
    robot_serve.move_L(middle_point, 1.5, 1.5)
    # Todo: go to label
    robot_serve.move_L(label_up, 2.5, 2.5)
    robot_serve.move_L(vis_label_pos, 2.5, 2.5)
    time.sleep(1.5)
    frame = camera_serve.get_frame()
    bgr_img = frame.color_image[0]
    # depth_img = frame.depth_image[0]
    grasp_label = VisLabel(ref_img, bgr_img)
    # go up
    robot_serve.move_L(label_up, 2.5, 2.5)
    robot_serve.move_L(middle_point, 1.5, 1.5)
    # release object
    robot_serve.move_L(up_pos, 1.5, 1.5)
    gripper_serve.set_tool_out(0, False)
    time.sleep(2)
    # go back home
    home_joints = np.array([-1.43131667, -1.20279444, -1.58378111, -1.90388667,  1.54697333, -1.42660667])
    robot_serve.move_j(home_joints, 2.8, 2.8)
    return grasp_label, bgr_img


class GraspCollection(object):
    def __init__(self, robot=None, camera=None):
        self.robot = robot
        self.camera = camera
        # row_min, row_max, col_min, col_max
        self.roi_img = [220, 490, 450, 790]
        # grid in pixel
        self.x_stride = 30  # col, pixel
        self.y_stride = 30  # row, pixel
        self.safe_dis_x = 20
        self.safe_dis_y = 20

        self.xlist = []
        self.ylist = []
        self.angle_map = []
        self.obj_class = "plastic"  # 'glass': 0, 'paper': 1, 'metal': 2, 'plastic': 3, 'Background': 4

    def generateGrid(self, xy_offset_flag=True):
        # grid in image
        self.xlist = np.arange(self.roi_img[2]+self.safe_dis_x, self.roi_img[3]-self.safe_dis_x, self.x_stride)
        self.ylist = np.arange(self.roi_img[0]+self.safe_dis_y, self.roi_img[1]-self.safe_dis_y, self.y_stride)
        # angle between row(y) and grasp orientation(line direction)
        self.angle_map = np.random.uniform(0, math.pi, size=[len(self.ylist), len(self.xlist)])

        # set random offset in grasping points
        if xy_offset_flag:
            pixel_jump_dis = 5
            pixel_offset_x = np.random.randint(-pixel_jump_dis, pixel_jump_dis, size=[len(self.xlist)])
            pixel_offset_y = np.random.randint(-pixel_jump_dis, pixel_jump_dis, size=[len(self.ylist)])
            self.xlist = self.xlist + pixel_offset_x
            self.ylist = self.ylist + pixel_offset_y

        return self.xlist, self.ylist, self.angle_map

    def vis(self, color_img):
        # show random grasping pose in image
        r_circle = 5
        for i in range(len(self.xlist)):
            for j in range(len(self.ylist)):
                temp_x = self.xlist[i]
                temp_y = self.ylist[j]
                cv2.circle(color_img, (temp_x, temp_y), 2, (238, 229, 0), 5)
                ptStart = (int(temp_x + r_circle * math.cos(self.angle_map[j, i])), int(temp_y + r_circle * math.sin(self.angle_map[j, i])))
                ptStop = (int(temp_x - r_circle * math.cos(self.angle_map[j, i])), int(temp_y - r_circle * math.sin(self.angle_map[j, i])))
                cv2.line(color_img, ptStart, ptStop, (0, 0, 255), thickness=2, lineType=3)
        cv2.imshow('test', color_img)
        cv2.waitKey()
        cv2.destroyAllWindows()

    def transfer2robot(self, i, j):
        path = './configs/ICRA2020-ur5-azure-rg6/cali2D_Azure.yaml'
        hand_eye = Calibration2D(path)
        temp_x = self.xlist[i]
        temp_y = self.ylist[j]
        robot_x, robot_y = hand_eye.cvt(temp_x, temp_y)
        robot_grasp_pose = [robot_x, robot_y, self.angle_map[j, i]]
        return robot_grasp_pose

    def collectGraping(self, saved_folder_path='./projects/ICRA2020/'):
        self.generateGrid()
        z_depth = 0.39
        # Go home before starting
        home_joints = np.array([-1.43131667, -1.20279444, -1.58378111, -1.90388667,  1.54697333, -1.42660667])
        self.robot.move_j(home_joints, 0.8, 0.8)
        self.robot.set_tool_out(0, False)
        time.sleep(1)
        # create folder for saving images
        folder_name = time.strftime("%Y%m%d%H%M%S", time.localtime())
        image_path = os.path.join(saved_folder_path, 'GraspingDate', folder_name, 'images')
        if not os.path.isdir(image_path):
            os.makedirs(image_path)
        label_path = os.path.join(saved_folder_path, 'GraspingDate', folder_name, 'labels')
        if not os.path.isdir(label_path):
            os.makedirs(label_path)
        print('saved image path: ', image_path)
        print('saved label path: ', label_path)
        # grasp and save data
        for i in range(len(self.xlist)):
            for j in range(len(self.ylist)):
                robot_grasp_pose = self.transfer2robot(i, j)
                label_dict = {}
                temp_pose = [robot_grasp_pose[0], robot_grasp_pose[1], z_depth, 3.14, 0, robot_grasp_pose[2]]
                # transfer to rotation vector
                r = R.from_euler('xyz', temp_pose[3:6], degrees=False)
                rvc = r.as_rotvec()
                pick_pose = [temp_pose[0], temp_pose[1], temp_pose[2], rvc[0], rvc[1], rvc[2]]
                # Todo: collect image
                """ get image"""
                frame = self.camera.get_frame()
                bgr_img = frame.color_image[0]
                depth_img = frame.depth_image[0]
                bgr_name = "{:08d}".format(i*len(self.xlist)+j) + '.jpg'
                depth_name = "{:08d}".format(i*len(self.xlist)+j) + '.npy'
                cv2.imwrite(image_path + '/' + bgr_name, bgr_img)
                np.save(image_path + '/' + depth_name, depth_img)
                # Todo: save label
                grasp_success, grasp_img = AutoGraspLabel(self.robot, self.robot, camera_serve=self.camera, ref_img=bgr_img, pick_xyzrt=pick_pose)
                label_img_name = "grasp_{:08d}".format(i*len(self.xlist)+j) + '.jpg'
                cv2.imwrite(image_path + '/' + label_img_name, grasp_img)

                label_dict.update({"imagePath": "../images/" + bgr_name})
                label_dict.update({"imageHeight": bgr_img.shape[0], "imageWidth": bgr_img.shape[1]})
                label_dict.update({"version": "4.5.6"})
                grasp_label = []
                single_grasp_label = {"label": self.obj_class, "position": [int(self.xlist[i]), int(self.xlist[j])], "angle": self.angle_map[j, i]}
                grasp_label.append(single_grasp_label)
                label_dict.update({"shapes": grasp_label})

                label_file = label_path + '/' + "{:08d}".format(i*len(self.xlist)+j) + '.json'
                with open(label_file, 'w') as f:
                    json.dump(label_dict, f, indent=4)


if __name__ == "__main__":
    # test = GraspCollection()
    # test.generateGrid()
    # color_img = cv2.imread('./deepclaw/modules/end2end/wasteNet/00000120.jpg')
    # print(color_img.shape[1])
    # xs = color_img.shape
    # print(xs[0])
    # test.vis(color_img)
    # xx = test.transfer2robot()
    # print(xx)

    robot = URController('./configs/robcell-ur5-rg6-d435/ur5.yaml')
    camera = AzureKinect()
    time.sleep(1)
    test = GraspCollection(robot, camera)
    test.collectGraping()




