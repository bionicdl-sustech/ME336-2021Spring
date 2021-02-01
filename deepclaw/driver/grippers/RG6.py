# Copyright (c) 2019 by liuxiaobo. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import socket
import sys
import os

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from driver.grippers.GripperController import GripperController


class RG6(GripperController):
    def __init__(self, robot_ip, port):
        super(RG6, self).__init__()
        self._robot_ip = robot_ip
        self._port = port

    def switch(self, pin, value):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self._robot_ip, self._port))
        command = f"set_digital_out({pin},{value})\n"
        command = bytes(command, encoding='utf-8')
        s.send(command)
        s.close()
