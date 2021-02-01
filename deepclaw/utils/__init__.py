# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/12/22
# -*- coding:utf-8 -*-

"""
@Modified:
@Description:
"""

import sys
import os
# append the DeepClawDev directory to python path and set it as working directory
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)
os.chdir(_root_path)
# print('work_dir: ', _root_path)