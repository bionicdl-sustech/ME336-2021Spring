# Copyright (c) 2020 By BionicDL Lab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: Math
@Author: Haokun Wang
@Date: 2020/3/20 14:16
@Description:
"""

import numpy as np
import math


def rpy2rotation(roll, pitch, yaw):
    yaw_matrix = np.matrix([[math.cos(yaw), -math.sin(yaw), 0],
                           [math.sin(yaw), math.cos(yaw), 0],
                           [0, 0, 1]])
    pitch_matrix = np.matrix([[math.cos(pitch), 0, math.sin(pitch)],
                             [0, 1, 0],
                             [-math.sin(pitch), 0, math.cos(pitch)]])
    roll_matrix = np.matrix([[1, 0, 0],
                            [0, math.cos(roll), -math.sin(roll)],
                            [0, math.sin(roll), math.cos(roll)]])
    R = yaw_matrix*pitch_matrix*roll_matrix
    theta = math.acos(((R[0, 0]+R[1, 1]+R[2, 2])-1)/2)
    multi = 1/(2*math.sin(theta))
    rx = multi*(R[2, 1]-R[1, 2])*theta
    ry = multi*(R[0, 2]-R[2, 0])*theta
    rz = multi*(R[1, 0]-R[0, 1])*theta
    return np.array([rx, ry, rz])
