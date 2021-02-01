# MIT License.
# Copyright (c) 2020 by BioicDL. All rights reserved.
# Created by LiuXb on 2020/11/24
# -*- coding:utf-8 -*-

"""
@Modified: 
@Description:
"""
import threading
import time
import queue
from deepclaw.driver.arms.URController_rtde import URController
import pickle


# receive
class GetRobotData(object):
    def __init__(self):
        self.flag = True

    def stop(self):
        self.flag = False

    # push data to buffer
    def run(self, robot: URController, data_buffer: queue.Queue):
        while self.flag:
            status = robot.get_state()
            time.sleep(0.01)
            time_stamp = time.time()
            status.update({'time': time_stamp})
            data_buffer.put(status)
            # print(data_buffer.get())


# write
class SaveRobotData(object):
    def __init__(self):
        self.flag = True

    def stop(self):
        self.flag = False

    def run(self, data_buffer: queue.Queue, filename: str):
        while self.flag:
            time.sleep(0.01)
            if data_buffer.empty():
                continue
            else:
                dd = data_buffer.get()
                with open(filename, "ab") as f:
                    pickle.dump(dd, f)


class MoveRobot(object):
    def __init__(self):
        self.flag = True
        self.action = None
        self.joint = None

    def stop(self):
        self.flag = False

    def set_joints(self, joint):
        self.joint = joint

    def run(self, robot: URController, data_buffer: queue.Queue = queue.Queue(maxsize=5000)):
        # get data
        gd = GetRobotData()
        read_thread = threading.Thread(target=gd.run, args=(robot, data_buffer,), daemon=True)
        read_thread.start()
        srd = SaveRobotData()
        write_thread = threading.Thread(target=srd.run, args=(data_buffer, 'test.result'), daemon=True)
        write_thread.start()
        # robot move
        robot.move_j(self.joint, 2.8, 2.2)
        gd.stop()
        srd.stop()


if __name__ == '__main__':
    rb = URController('../../configs/robcell-ur5-rg6-d435/ur5.yaml')
    print('Start move!')
    joints_pos = [-1.41319307, -1.51162964, -1.66329875, -1.50447379,  1.53746051, 0.14490873]
    db = queue.Queue(maxsize=0)
    x = MoveRobot()
    x.set_joints(joints_pos)
    x.run(rb, db)
    # state = robot.get_state()
    # print(state)
    rb.go_home()
    print('reach home pose')

    # for i in range(10):
    #     status = robot.get_state()
    #     time_stamp = time.time()
    #     status.update({'time': time_stamp})
    #     print(status)
    #     time.sleep(0.5)
    #     with open("dict", "ab") as f:
    #         pickle.dump(status, f)
    #
    print('============================================')
    with open("test.result", 'rb') as f:
        while True:
            try:
                aa = pickle.load(f)
                print(aa)
            except EOFError:
                break














