import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #für 6D Transformation 
import numpy as np


def complex_movement_example(robot, pose):
    offset = Affine((0, 0, 0.1))
    new_pose = offset * pose
    robot.lin(new_pose)

    robot.ptp(pose)


def main(args=None):
    # initialize ros communications for a given context
    rclpy.init(args=args)

    # initialize robot client node
    # if not connected to the real robot set is_simulation=True
    robot = RobotClient(is_simulation=True)
    
    # move robot to home position
    robot.home()

    # get endeffector pose (returns affine transformation/ kinda same than homogen transformations)
    current_pose = robot.node.get_transform('tcp_link', 'world')
    print(current_pose)

    # robot ptp movement to given joint positions
    robot.ptp_joint([np.pi / 4, -np.pi / 2, np.pi / 2, 0.0, -np.pi / 4, 0.0])


    # # robot ptp movement to given cartesian pose                                                
    # # first tuple represents cartesian coordinates (x, y, z), the second tuple represents rotation in quaternions (x, y, z, w)
    # robot.ptp(Affine((0.03607227, 0.15249834, 1.20871037),
    #                  (3.80250600e-05, 9.89829634e-01, 4.76747912e-05, 1.42257839e-01)))
    # current_pose = robot.node.get_transform('tcp_link', 'world')
    # print(current_pose)
    # # open gripper (does nothing in simulation)
    # robot.open_gripper()
    # # robot ptp movement again; you can also use RPY angles instead of quaternions (alpha, delta, theta)
    # robot.ptp(Affine((-0.151, 0.193, 1.111), (np.pi, 0, -np.pi / 2)))
    # # robot lin movement to given pose
    # robot.lin(Affine((-0.051, 0.193, 1.111), (np.pi, 0, -np.pi / 2)))
    # # close gripper (does nothing when in simulation)
    # robot.close_gripper()
    # back to home pose
    # robot.home()

    # # translation of the tcp 
    # # translation in tcp coordinate system (-0.05 m in z_tcp direction)                          -> relative Transformation in tcp-KOS
    # movement_tcp = Affine((0, 0, -0.05))
    # # apply translation to current pose (home) given in world coordinates
    # current_pose = robot.node.get_transform('tcp_link', 'world')
    # pose = current_pose * movement_tcp
    # robot.lin(pose)

    # # translation in world coordinate system (0.1 in z_world direction)                         -> relative Transformation in world KOS
    # movement_world = Affine((0, 0, 0.1))
    # # apply translation to current pose given in world coordinates
    # pose = movement_world * pose
    # robot.lin(pose)

    # # you can define more complex movements using methods (Def siehe oben)
    # current_pose = robot.node.get_transform('tcp_link', 'world')
    # complex_movement_example(robot, current_pose)

    # destroy the robot node
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()
