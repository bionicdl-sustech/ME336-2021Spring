# MIT License.
# Copyright (c) 2021 by BioicDL. All rights reserved.
# Created by LiuXb on 2021/2/27
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description: check the device port "ls /dev/ttyUSB*", for example the device is ttyUSB0.
"sudo chmod 777 /dev/ttyUSB0" to set permissions of ttyUSB0.

"""

"""
安装serial串口通迅模块：
pip install pyserial
查询CH340单片机模块是否驱动
1.首先确认系统支持USBSerial，输入以下命令：
     lsmod | grep usbserial
2.接上USB串口线，看看系统是否可以识别。输入以下命令：
     dmesg | grep ttyUSB
     或者直接可以到/ dev下看看有没有ttyUSB0或ttyUSB1
单片机模板:CH340
"""

import serial


class ComSwitch(object):
    def __init__(self, com="/dev/ttyUSB0"):
        self.__open_cmd = [0xA0, 0x01, 0x01, 0xA2]
        self.__close_cmd = [0xA0, 0x01, 0x00, 0xA1]
        self.sc = serial.Serial(port=com, baudrate=9600)

    def open(self):
        self.sc.write(self.__open_cmd)

    def close(self):
        self.sc.write(self.__close_cmd)


if __name__ == "__main__":
    print('CH430 serial test!')
    import time

    cs = ComSwitch()
    for i in range(5):
        cs.open()
        time.sleep(0.5)
        cs.close()
        time.sleep(0.5)
