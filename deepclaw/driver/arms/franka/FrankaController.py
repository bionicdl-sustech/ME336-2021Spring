
import time
import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
sys.path.append(ROOT)

from deepclaw.driver.arms.ArmController import ArmController
from deepclaw.driver.arms.URConnector import URConnector


import yaml
import numpy as np
from scipy.spatial.transform import Rotation as RR
import RobotDriver

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


class FrankaController(ArmController):
    def __init__(self, robot_configuration_file_path):
        super(FrankaController, self).__init__()
        self._cfg = yaml.load(open(robot_configuration_file_path,'r'),Loader=yaml.FullLoader)
        robot_ip = self._cfg['ip']
        self.fk = RobotDriver.FrankaDriver(robot_ip)
        #F_T_EE: 4*4 Matrix from Flange to EE
        # this is setted in website
        F_T_EE = np.array(self.fk.getF_T_EE())
        self.F_Matrix = np.reshape(F_T_EE,(4,4),order='F')
        self.v = self._cfg['velocity']
        self.fk.setPositioningVelocity(self.v)


    def getCartesianPose(self):
        #O_T_EE: 4*4 Matrix from Base to EE
        O_T_EE = np.array(self.fk.getO_T_EE())
        O_Matrix = np.reshape(O_T_EE,(4,4),order='F')
        O_rotation = RR.from_matrix(O_Matrix[0:3, 0:3])
        O_euler = O_rotation.as_euler('xyz', degrees=False)

        # #F_T_EE: 4*4 Matrix from Flange to EE
        # F_T_EE = np.array(self.fk.getF_T_EE())
        # F_Matrix = np.reshape(F_T_EE,(4,4),order='F')
        return O_Matrix[0:3, 3], O_euler

    def getJoint(self):
        return self.fk.getJointPos()

    def get_state(self):
        current_state = self.fk.getAllState()
        current_state = eval(current_state)
        
        robot_state["Joints"] = current_state['q']
        robot_state["Joints_Velocity"] = current_state['dq']
        robot_state["Joints_Acceleration_Target"] = current_state['ddq_d']
        
        temp = current_state['O_T_EE']
        O_Matrix = np.reshape(temp,(4,4),order='F')
        O_rotation = RR.from_matrix(O_Matrix[0:3, 0:3])
        O_euler = O_rotation.as_euler('xyz', degrees=False)
        xyz = O_Matrix[0:3, 3]
        robot_state["TCP_Pose"] = [xyz[0], xyz[1], xyz[2], O_euler[0], O_euler[1], O_euler[2]]

        return robot_state

    def gripperOpen(self):
        self.fk.gripperOpen()

    def gripperGrasp(self,width=0.05,force=1):
        self.fk.gripperGrasp(width,force)

        # rad/s
    def speed_j(self,joint_speed):
        self.fk.speedJ(joint_speed)

    def stopSpeed(self):
        self.fk.stopMotion()

    def move_j(self, joints_angle, velocity=0.5, accelerate=0.6, solution_space='Joint'):
        self.fk.setJointPos(joints_angle)

    def move_p(self,position, velocity=0.5, accelerate=0.6, solution_space='Joint'):
        # frome euler to Matrix, TCP
        target_pose = np.array(position)
        temp_euler = RR.from_euler('xyz', target_pose[3:6], degrees=False)
        temp_matrix = temp_euler.as_matrix()
        target_matrix = np.eye(4)
        target_matrix[0:3,0:3] = temp_matrix
        target_matrix[0:3,3] = target_pose[0:3]
        # TCP to Flange
        O_T_F = np.dot(target_matrix,np.linalg.pinv(self.F_Matrix))

        # inverse kinematics
        sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/Inverse_Kinematics')
        import ikpy

        link_used = [False, True, True, True,True,True,True,False]
        robot_urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/panda.urdf"
        my_chain = ikpy.chain.Chain.from_urdf_file(robot_urdf_path, active_links_mask = None)
        # base_joint and gripper_joint, so there are 9 joint angles
        current_joint = self.getJoint()
        init_joint = np.zeros(9)
        init_joint[1:8] = current_joint
        ik_joint = my_chain.inverse_kinematics(O_T_F,init_joint)
        target_joint = ik_joint[1:8]

        #
        self.fk.setJointPos(target_joint)

if __name__ == '__main__':
    FC = FrankaController(ROOT + '/configs/robcell-panda1-default-d435/'+'franka.yaml')
    print(FC.getJoint())
    #print(FC.getCartesianPose())
    allState = FC.getState()
    print('state: ', allState)

    joint_target = np.array([-0.0137566,0.0150639,0.06416,-2.50988,-0.00736516,2.80153,-1.8411])
    FC.move_j(joint_target)
    pose = [0.5,0,0.4,3.14,0.0,0.0]
    FC.move_p(pose)
    # speed_j
    joint_speed = [0,0,0,0,0,0,0.1]
    FC.speed_j(joint_speed)
    time.sleep(2)
    FC.stopSpeed()
    print('===========')

    FC.move_p([0.5,0,0.3,3.14,0.0,0.0])
    FC.move_p([0.6,0,0.4,3.14,0.0,1.0])
    FC.move_p([0.6,0.3,0.2,3.14,0.0,1.0])
