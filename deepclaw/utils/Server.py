# Copyright (c) 2020 by BionicLab. All Rights Reserved.
# !/usr/bin/python3
# -*- coding:utf-8 -*-
"""
@File: Server
@Author: Haokun Wang
@Date: 2020/3/23 14:31
@Description: 
"""

from socket import *
import struct
import json
from .IO import JsonEncoder


class Server(object):
    def __init__(self, host_ip, port):
        self.flag = True
        self.code = 'utf-8'
        self.host_ip = host_ip
        self.port = port
        self.connections = 10
        self.tcp_socket = socket(AF_INET, SOCK_STREAM)

        self.instance = None

    def start(self):
        self.tcp_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)  # reuse ip and port
        self.tcp_socket.bind((self.host_ip, self.port))
        self.tcp_socket.listen(self.connections)  # the maximal connections

    def send(self, data: dir):
        data_info = json.dumps(data, cls=JsonEncoder)
        data_info_len = struct.pack('i', len(data_info))
        self.tcp_socket.send(data_info_len)
        self.tcp_socket.send(data_info.encode(self.code))

    def send2client(self, data: dir, client: socket):
        data_info = json.dumps(data, cls=JsonEncoder)
        data_info_len = struct.pack('i', len(data_info))
        client.send(data_info_len)
        client.send(data_info.encode(self.code))

    def get_instance(self, instruction):
        exec(instruction[0])  # from modules.xxxx import xxxx
        args = instruction[1]  # parameters in list
        if len(args) == 0:
            self.instance = eval(instruction[2] + '()')  # self.instance = xxxx(*args)
        elif len(args) == 1:
            args = args[0]
            self.instance = eval(instruction[2] + '(args)')  # self.instance = xxxx(*args)
        else:
            args = args[0]
            self.instance = eval(instruction[2]+'(*args)')  # self.instance = xxxx(*args)

    def handle(self, client):
        data_struct = client.recv(4)  #
        data_len = struct.unpack('i', data_struct)[0]
        data = client.recv(data_len, MSG_WAITALL)
        data_dir = json.loads(data.decode(self.code))
        if data_dir['type'] == 'instruction':
            self.get_instance(data_dir['data'])
        else:
            if self.instance is not None:
                self.send2client({'feedback': self.instance.run(data_dir['data'])}, client)

    def run(self):
        print('Server starting...')
        self.start()
        while self.flag:
            connected_client, address = self.tcp_socket.accept()
            print("Connected client: ", address)
            self.handle(connected_client)
            connected_client.close()  # close connection

    def stop(self):
        self.flag = False
        self.tcp_socket.close()
        print('Server stopped.')
