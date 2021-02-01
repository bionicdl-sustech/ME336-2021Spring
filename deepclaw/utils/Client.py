# Copyright (c) 2020 by BionicLab. All Rights Reserved.
# !/usr/bin/python
# -*- coding:utf-8 -*-
"""
@File: Client
@Author: Haokun Wang
@Date: 2020/3/23 16:03
@Description:
"""

from socket import *
import struct
import json
from .IO import JsonEncoder


class Client(object):
    def __init__(self, host_ip, port):
        self.flag = True
        self.code = 'utf-8'
        self.host_ip = host_ip
        self.port = port
        self.client_socket = socket(AF_INET, SOCK_STREAM)

    def start(self):
        self.client_socket = socket(AF_INET, SOCK_STREAM)
        self.client_socket.connect((self.host_ip, self.port))

    def send(self, data: dir):
        data_info = json.dumps(data, cls=JsonEncoder)
        data_info_len = struct.pack('i', len(data_info))
        self.client_socket.send(data_info_len)
        self.client_socket.send(data_info.encode(self.code))

    def recv(self):
        data_struct = self.client_socket.recv(4)  #
        data_len = struct.unpack('i', data_struct)[0]
        data = self.client_socket.recv(data_len, MSG_WAITALL)
        data_dir = json.loads(data.decode(self.code))
        return data_dir

    def close(self):
        self.client_socket.close()
