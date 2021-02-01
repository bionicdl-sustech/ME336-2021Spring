# Copyright (c) 2020 by BionicDL Lab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: UR5Controller
@Author: Haokun Wang
@Date: 2020/3/19 11:09
@Description:
"""

import yaml
import json
import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(ROOT)


def read_yaml(file_name):
    yaml_file = open(ROOT+file_name)
    return yaml.load(yaml_file)


class JsonEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)