# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/5/11
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: control UR with RTDE, and the port is fixed in 30004.
The offical description is in https://www.universal-robots.com/articles/ur-articles/real-time-data-exchange-rtde-guide/
and the rtde usage is in https://sdurobotics.gitlab.io/ur_rtde/index.html
"""

import sys
import os
import time
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(ROOT)

from deepclaw.driver.arms.ArmController import ArmController

import yaml
# from scipy.spatial.transform import Rotation as RR

import rtde_receive
import rtde_control
import rtde_io


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


class URController(ArmController):
    def __init__(self, robot_configuration_file_path):
        super(URController, self).__init__()
        self._cfg = yaml.load(open(robot_configuration_file_path, 'r'), Loader=yaml.FullLoader)
        self._home_joints = self._cfg['home_joint']
        self._home_pose = self._cfg['home_pose']
        self._ip = self._cfg['ip']
        self._port = self._cfg['port']
        self._v = self._cfg['velocity']
        self._a = self._cfg['acceleration']

        # create connection
        self.rtde_R = rtde_receive.RTDEReceiveInterface(self._ip)
        self.rtde_C = rtde_control.RTDEControlInterface(self._ip)
        self.rtde_IO = rtde_io.RTDEIOInterface(self._ip)

    def go_home(self):
        joint = [self._home_joints[0], self._home_joints[1], self._home_joints[2],
                 self._home_joints[3], self._home_joints[4], self._home_joints[5]]
        self.move_j(joint)

    '''
    @Description: go to the target joints positions
    @param[in]: joints_angle,target joint positions  of each joints or pose;
                velocity:joint acceleration of leading axis [rad/s]
                accelerate: joint speed of leading axis [rad/^2]
                solution_space: move style, 'Joint' means it linear in joint-space,
                                and 'Space' means linear in tool-space(forward kinematics is used to calculate the corresponding pose)

    @return: bool, reaching target or not
    '''

    def move_j(self, joints_angle, velocity=None, acceleration=None,
               solution_space='Joint'):
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration
        if solution_space == 'Joint':
            self.rtde_C.moveJ(joints_angle, velocity, acceleration)
        elif solution_space == 'Space':
            self.rtde_C.moveJ_IK(joints_angle, velocity, acceleration)
        return True

    '''
    @Description: go to the target pose
    @param[in]: position,target position and rotation vector [x,y,z,rx,ry,rz];
                velocity:joint acceleration of leading axis [rad/s]
                accelerate: joint speed of leading axis [rad/s^2]
                solution_space: move style, 'Joint' means it linear in joint-space(inverse kinematics is used to calculate the corresponding joints),
                                and 'Space' means linear in tool-space

    @return: bool, reaching target or not
    '''

    def move_p(self, position, velocity=None, acceleration=None,blend = 0.0,
               solution_space='Space'):
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration
        if solution_space == 'Joint':
            print('Please use move_j!')
        elif solution_space == 'Space':
            self.rtde_C.moveP(position, velocity, acceleration, blend)

    def move_L(self, position, velocity = None, acceleration = None, solution_space='Space'):
        velocity = self._v if velocity is None else velocity
        acceleration = self._a if acceleration is None else acceleration
        if solution_space == 'Joint':
            self.rtde_C.moveL_FK(position, velocity, acceleration)
        elif solution_space == 'Space':
            if len(np.array(position).shape) == 1 and len(position) == 6:
                self.rtde_C.moveL(position, velocity, acceleration)
            elif len(np.array(position).shape) == 2 and len(position[0]) == 9:
                self.rtde_C.moveL(position)
        return True

    def speed_L(self, pos_speed=None, acceleration=0.5, dt=0.0):
        t1 = time.time()
        self.rtde_C.speedL(pos_speed, acceleration, dt)
        t2 = time.time()
        duration = t2 - t1
        if duration < dt:
            time.sleep(dt - duration)
        return True

    def speed_J(self, joints_speed, acceleration=0.5, dt=0.0):
        t1 = time.time()
        self.rtde_C.speedJ(joints_speed, acceleration, dt)
        t2 = time.time()
        duration = t2 - t1
        if duration < dt:
            time.sleep(dt - duration)
        return True

    def stop_speed(self):
        self.rtde_C.speedStop()

    # get robot state
    def get_state(self):
        robot_state['Joints'] = self.rtde_R.getActualQ()
        robot_state['Joints_Velocity'] = self.rtde_R.getActualQd()
        robot_state['TCP_Force'] = self.rtde_R.getActualTCPForce()
        robot_state['TCP_Pose'] = self.rtde_R.getActualTCPPose()
        robot_state['TCP_Velocity'] = self.rtde_R.getActualTCPSpeed()
        robot_state['Joints_Current'] = self.rtde_R.getActualCurrent()
        robot_state['Joints_Voltage'] = self.rtde_R.getActualJointVoltage()
        robot_state['Joints_Temperature'] = self.rtde_R.getJointTemperatures()
        robot_state['Joints_Target'] = self.rtde_R.getTargetQ()
        robot_state['Joints_Velocity_Target'] = self.rtde_R.getTargetQd()
        robot_state['Joints_Acceleration_Target'] = self.rtde_R.getTargetQdd()
        robot_state['Joints_Current_Target'] = self.rtde_R.getTargetCurrent()
        robot_state['Joints_Torque_Target'] = self.rtde_R.getTargetMoment()
        robot_state['Joints_Current_Control'] = self.rtde_R.getJointControlOutput()
        robot_state['TCP_Acceleration'] = self.rtde_R.getActualToolAccelerometer()
        robot_state['TCP_Pose_Target'] = self.rtde_R.getTargetTCPPose()
        robot_state['TCP_Velocity_Target'] = self.rtde_R.getTargetTCPSpeed()
        robot_state['Speed_Fraction_Target'] = self.rtde_R.getTargetSpeedFraction()
        robot_state['Speed_Scaling'] = self.rtde_R.getSpeedScaling()
        robot_state['Momentum'] = self.rtde_R.getActualMomentum()
        # robot_state['isProtectiveStopped'] = self.rtde_R.isProtectiveStopped()
        return robot_state

    # set io
    def set_digital_out(self, output_id=7, signal=True):
        self.rtde_IO.setStandardDigitalOut(output_id, signal)

    def set_tool_out(self, output_id=0, signal=True):
        self.rtde_IO.setToolDigitalOut(output_id, signal)

    def test(self):
        a = self.rtde_C.isConnected()
        return a


if __name__ == '__main__':
    robot = URController('../../../configs/robcell-ur5-rg6-d435/ur5.yaml')
    # print('Start move!')
    # robot.go_home()
    # print('reach home pose')
    # joints_pos = [-1.41319307, -1.51162964, -1.66329875, -1.50447379,  1.53746051, 0.14490873]
    # robot.move_j(joints_pos, 0.8, 1.2)
    # for RG6
    # robot.set_tool_out(0, True)
    state = robot.get_state()
    print(state)

