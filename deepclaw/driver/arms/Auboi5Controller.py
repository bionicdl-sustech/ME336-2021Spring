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
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(ROOT)


from deepclaw.driver.arms.aubo.robotcontrol import Auboi5Robot, RobotErrorType
# from robotcontrol import Auboi5Robot
from deepclaw.driver.arms.ArmController import ArmController


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
        # rpy: euler angle（rx, ry, rz）;unit: degree
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration

        self.robot.set_end_max_line_acc(acceleration)
        self.robot.set_end_max_line_velc(velocity)
        self.robot.move_to_target_in_cartesian(position[0:3], position[3:6])
        return True

    def get_state(self, *args, **kwargs):
        self.robot.get_robot_state()
        kk = self.robot.get_joint_status()
        print(kk)

    def verify_state(self, *args, **kwargs):
        pass


if __name__ == "__main__":
    os.chdir(ROOT)
    ss = AuboController('./configs/basic_config/robot_auboi5.yaml')
    home_joint = [0, 0, 1.57, 0, 1.57, 0]
    ss.move_j(home_joint, 1.5, 1.5)
    time.sleep(0.01)
    position = [0.470, -0.12, 0.212, 180, 0, 90]
    ss.move_p(position, 5, 5)
    ss.get_state()









