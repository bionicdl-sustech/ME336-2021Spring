# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/1/20
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""

import sys
import os
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(ROOT)

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
from deepclaw.driver.arms.aubo.robotcontrol import Auboi5Robot, RobotErrorType
# from robotcontrol import Auboi5Robot
from deepclaw.driver.arms.ArmController import ArmController


robot_state = {'Joints': [],  # Actual joint positions
               'Joints_Velocity': [],  # Actual joint velocities
               'Joints_Current': [],  # Actual joint currents
               'Joints_Voltage': [],  # Actual joint voltages
               'Joints_Temperature': [],  # Temperature of each joint in degrees Celsius
               'Joints_Target': [],  # Target joint positions
               'Joints_Velocity_Target': [],  # Target joint velocities
               'Joints_Acceleration_Target': [],  # Target joint accelerations
               'Joints_Current_Target': [],  # Target joint currents
               'Joints_Torque_Target': [],  # Target joint moments (torques)
               'Joints_Current_Control': [],  # Joint control currents
               'TCP_Pose': [],  # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz)
               'TCP_Force': [],  # Generalized forces in the TCP
               'TCP_Velocity': [],  # Actual speed of the tool given in Cartesian coordinates
               'TCP_Acceleration': [],  # Tool x, y and z accelerometer values
               'TCP_Pose_Target': [],  # Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz)
               'TCP_Velocity_Target': [],  # Target speed of the tool given in Cartesian coordinates
               'Speed_Fraction_Target': [],  # Target speed fraction, running speed fraction in teach pad
               'Speed_Scaling': [],  # Speed scaling of the trajectory limiter
               'Momentum': [],  # Norm of Cartesian linear momentum
               'isProtectiveStopped': []  # a bool indicating if the robot is in ‘Protective stop’
               }


class AuboController(ArmController):
    def __init__(self, robot_configuration_file_path):
        super(AuboController, self).__init__()
        self._cfg = yaml.load(open(robot_configuration_file_path, 'r'), Loader=yaml.FullLoader)
        self._home_joints = self._cfg['home_joint']
        self._home_pose = self._cfg['home_pose']
        self._ip = self._cfg['ip']
        self._port = self._cfg['port']
        self._v = self._cfg['velocity']
        self._a = self._cfg['acceleration']

        #
        Auboi5Robot.initialize()
        self.robot = Auboi5Robot()
        handle = self.robot.create_context()
        # connect robot

        result = self.robot.connect(self._ip, self._port)
        if result != RobotErrorType.RobotError_SUCC:
            print("connect server{0}:{1} failed.".format(self._ip, self._port))
        else:
            self.robot.robot_startup()
            self.robot.set_collision_class(7)

    def move_j(self, joints_angle, velocity=None, acceleration=None,
               solution_space='Joint'):
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration
        self.robot.set_joint_maxacc((acceleration, acceleration, acceleration, acceleration, acceleration, acceleration))
        self.robot.set_joint_maxvelc((velocity, velocity, velocity, velocity, velocity, velocity))

        self.robot.move_joint(joints_angle)
        return True

    def move_p(self, position, velocity=None, acceleration=None, blend = 0.0,
               solution_space='Space'):
        # position = [x,y,z,r, p, y]
        # pos: position（x, y, z）;unit: m
        # rpy: euler angle（rx, ry, rz）;unit: rad
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration

        self.robot.set_end_max_line_acc(acceleration)
        self.robot.set_end_max_line_velc(velocity)
        rx = position[3] * 180.0 / 3.14159
        ry = position[4] * 180.0 / 3.14159
        rz = position[5] * 180.0 / 3.14159
        self.robot.move_to_target_in_cartesian(position[0:3], [rx, ry, rz])
        return True

    def get_state(self):
        gear_state = self.robot.get_joint_status()
        robot_state['Joints_Current'] = [gear_state['joint1']['current'], gear_state['joint2']['current'],
                                         gear_state['joint3']['current'], gear_state['joint4']['current'],
                                         gear_state['joint5']['current'], gear_state['joint6']['current']]

        robot_state['Joints_Voltage'] = [gear_state['joint1']['voltage'], gear_state['joint2']['voltage'],
                                         gear_state['joint3']['voltage'], gear_state['joint4']['voltage'],
                                         gear_state['joint5']['voltage'], gear_state['joint5']['voltage']]
        robot_state['Joints_Temperature'] = [gear_state['joint1']['temperature'], gear_state['joint2']['temperature'],
                                             gear_state['joint3']['temperature'], gear_state['joint4']['temperature'],
                                             gear_state['joint5']['temperature'], gear_state['joint6']['temperature']]
        temp = self.robot.get_current_waypoint()
        robot_state['Joints'] = temp['joint']
        tcp_xyz = temp['pos']
        tcp_ori = temp['ori']

        reu = self.robot.quaternion_to_rpy(tcp_ori)

        robot_state['TCP_Pose'] = [tcp_xyz[0], tcp_xyz[1], tcp_xyz[2], reu[0], reu[1], reu[2]]
        return robot_state

    def verify_state(self, *args, **kwargs):
        pass


if __name__ == "__main__":
    os.chdir(ROOT)
    ss = AuboController('./configs/basic_config/robot_auboi5.yaml')
    home_joint = [0, 0, 1.57, 0, 1.57, 0]
    ss.move_j(home_joint, 1.5, 1.5)
    time.sleep(0.01)
    position = [0.470, -0.12, 0.212, 3.14159, 0, 0]
    ss.move_p(position, 5, 5)
    ss.get_state()









