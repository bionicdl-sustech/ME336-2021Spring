from robotcontrol import Auboi5Robot
import time

Auboi5Robot.initialize()
robot = Auboi5Robot()
handle = robot.create_context()
ip = '192.168.0.100'
port = 8899
result = robot.connect(ip, port)
robot.robot_startup()
robot.set_collision_class(7)
joint_status = robot.get_joint_status()
#print(joint_status)
robot.set_joint_maxvelc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))

xyz = [0.367, -0.344, 0.435]
rpy = [180, 0, 0]
xyz_1 = [0.367, -0.344, 0.235]

for i in range(5):
    robot.move_to_target_in_cartesian(xyz, rpy)
    time.sleep(3)
    robot.move_to_target_in_cartesian(xyz_1, rpy)
    time.sleep(3)

