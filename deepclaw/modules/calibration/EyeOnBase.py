# Copyright (c) 2020 by Hank. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: EyeOnBase
@Author: Haokun Wang
@Date: 2020/3/18 10:46
@Description: 
"""
import numpy as np
import cv2
import os
import sys
import yaml
from scipy.spatial.transform import Rotation as R
root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(root_path)
os.chdir(root_path)


def image_callback(color_image, depth_image, intrinsics):
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    fx = intrinsics[0]
    fy = intrinsics[1]
    cx = intrinsics[2]
    cy = intrinsics[3]

    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None,
                                                            cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
        # the unit of depth in camera is meter
        checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1] - 20:checkerboard_pix[1] + 20,
                                         checkerboard_pix[0] - 20:checkerboard_pix[0] + 20]))
        checkerboard_x = np.multiply(checkerboard_pix[0] - cx, checkerboard_z / fx)  # 1920, 1080
        checkerboard_y = np.multiply(checkerboard_pix[1] - cy, checkerboard_z / fy)  # 1920, 1080
        print("Found checkerboard, X,Y,Z = ", [checkerboard_x, checkerboard_y, checkerboard_z])
        if checkerboard_z > 0:
            # Save calibration point and observed checkerboard center
            observed_pt = np.array([checkerboard_x, checkerboard_y, checkerboard_z])
            return observed_pt
    return []


# main_sensor: 'depth', 'color', 'infrared'
class Calibration(object):
    def __init__(self, arm, camera, configuration_file, main_sensor='color'):
        self.__arm = arm
        self.__camera = camera
        self.__cfg = yaml.load(open(configuration_file, 'r'), Loader=yaml.FullLoader)
        self.__sensor = main_sensor

    def run(self):
        initial_pose = self.__cfg['initial_position']
        x_step = self.__cfg['x_stride']
        y_step = self.__cfg['y_stride']
        z_step = self.__cfg['z_stride']

        self.__arm.move_p(initial_pose)
        x = initial_pose[0]
        y = initial_pose[1]
        z = initial_pose[2]

        observed_pts = []
        measured_pts = []
        sensor_intrinsics = None
        if self.__sensor == 'depth':
            temp = self.__camera.get_intrinsics()
            if len(temp) == 5:
                sensor_intrinsics = temp
            elif len(temp) == 3:
                # the old api is temp[2].distCoeffs
                sensor_intrinsics = (temp[1].fx, temp[1].fy, temp[1].ppx, temp[1].ppy, temp[1].coeffs)
        elif self.__sensor == 'color':
            temp = self.__camera.get_intrinsics()
            if len(temp) == 5:
                sensor_intrinsics = temp
            elif len(temp) == 3:
                sensor_intrinsics = (temp[0].fx, temp[0].fy, temp[0].ppx, temp[0].ppy, temp[0].coeffs)
        elif self.__sensor == 'infrared':
            temp = self.__camera.get_intrinsics()
            if len(temp) == 5:
                sensor_intrinsics = temp
            elif len(temp) == 3:
                sensor_intrinsics = (temp[2].fx, temp[2].fy, temp[2].ppx, temp[2].ppy, temp[2].coeffs)
        else:
            print('Error input! The Sensor must be \'depth\', \'color\', or \'infrared\' ')

        for i in range(4):
            for j in range(4):
                for k in range(4):
                    self.__arm.move_p([x + x_step * i, y + y_step * j, z + z_step * k,
                                       initial_pose[3], initial_pose[4], initial_pose[5]])
                    # time.sleep(0.5)
                    frame = self.__camera.get_frame()
                    color_image = frame.color_image[0]
                    depth_image = frame.depth_image[0]
                    observed_pt = image_callback(color_image, depth_image, sensor_intrinsics)

                    current_pose = self.__arm.get_state()['TCP_Pose']
                    # current_matrix = rotvec2matrix(current_pose)
                    current_matrix = rpy2matrix(current_pose)
                    transfer_matrix = self.__cfg['E_T_F']
                    transfer_matrix = np.array(transfer_matrix)
                    end_in_base = np.dot(current_matrix, transfer_matrix)
                    measured_pt = [end_in_base[0][3], end_in_base[1][3], end_in_base[2][3]]
                    print(measured_pt)
                    if len(observed_pt) != 0:
                        observed_pts.append(observed_pt)
                        measured_pts.append(measured_pt)
        # save data
        np.savez(self.__cfg["CALIBRATION_DIR"], observed_pts, measured_pts)


def load_calibration_matrix(file):
    d = np.load(file)
    observed_pts = d['arr_0']
    measured_pts = d['arr_1']
    R, t = get_rigid_transform(observed_pts, measured_pts)
    H = np.concatenate([np.concatenate([R,t.reshape([3,1])],axis=1),np.array([0, 0, 0, 1]).reshape(1,4)])
    return H


def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]  # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1))  # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB)  # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:  # Special reflection case
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t


def rotvec2matrix(xyzrpy):
    r = R.from_rotvec(xyzrpy[3:6])
    temp_matrix = np.eye(4)
    rotation = r.as_matrix()
    temp_matrix[0:3, 0:3] = rotation
    temp_matrix[0:3, 3] = xyzrpy[0:3]
    return temp_matrix

def rpy2matrix(xyzrpy):
    r = R.from_euler('xyz', xyzrpy[3:6])
    temp_matrix = np.eye(4)
    rotation = r.as_matrix()
    temp_matrix[0:3, 0:3] = rotation
    temp_matrix[0:3, 3] = xyzrpy[0:3]
    return temp_matrix


def matrix2rv(matrix4):
    rotation = np.eye(3)
    rotation = matrix4[0:3, 0:3]
    r = R.from_matrix(rotation)
    rotation_vector = r.as_rotvec()
    temp = np.zeros(6)
    temp[0:3] = matrix4[0:3, 3]
    temp[3:6] = rotation_vector
    return temp


if __name__ == "__main__":
    collect_flag = False
    calculate_flag = True
    if collect_flag:
        # collect calibration data
        from deepclaw.driver.sensors.camera.AzureKinect import AzureKinect
        from deepclaw.driver.sensors.camera.Realsense_L515 import Realsense
        from deepclaw.driver.arms.URController_rtde import URController
        import time
        camera = AzureKinect()
        # camera = Realsense('./configs/basic_config/camera_rs.yaml')
        time.sleep(1)
        frame = camera.get_frame()
        color_img = frame.color_image[0]
        depth_img = frame.depth_image[0]
        # calibrate
        robot = URController('./configs/basic_config/robot_ur5.yaml')
        calib_ins = Calibration(robot, camera, './configs/ICRA2020-ur5-azure-rg6/calib_cfg.yaml', main_sensor='color')
        calib_ins.run()

    if calculate_flag:
        c_t_b = load_calibration_matrix('./configs/ICRA2020-ur5-azure-rg6/calib_Azure.npz')
        np.save('./configs/ICRA2020-ur5-azure-rg6/E_T_B_Azure.npy', c_t_b)
        print(c_t_b)
