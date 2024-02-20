import rclpy
from ros_environment.scene import RobotClient
from manipulation_tasks.transform import Affine #für 6D Transformation 
import numpy as np



def main(args=None):
    # initialize ros communications for a given context
    rclpy.init(args=args)

    # initialize robot client node
    # if not connected to the real robot set is_simulation=True
    robot = RobotClient(is_simulation=True)
    # home joint positions of the robot in Jointroom (Winkel in Rad default)
    robot.home_position = [0.0, -np.pi / 2, np.pi / 2, 0.0, np.pi / 4, 0.0]
    # move robot to home position (Methode im Robot Client Libary)
    robot.home()
    # get endeffector pose (tcp abfragen)
    current_pose = robot.node.get_transform('tcp_link', 'world')
    print(current_pose)

    # robot ptp movement to house start -> absolute Transformation  stimmt nicht!!!        
    movement_world = Affine((0, -0.15, -0.15))
    newpose = movement_world * current_pose
    robot.lin(newpose)
 
   # robot.ptp(Affine((0.1360703,  -0.05248998,  1.30864724),(-1.70511577e-05  9.89837445e-01 -2.64399931e-05  1.42203485e-01)))


    #robot lin movement nach oben rechts -> relative im world-KOS
    movement_world = Affine((0, 0.2, 0.2))
    current_pose = robot.node.get_transform('tcp_link', 'world')
    newpose = movement_world * current_pose
    robot.lin(newpose)

    #robot lin movement nach oben links -> relative im world-KOS
    movement_world = Affine((0, -0.2, 0))
    current_pose = robot.node.get_transform('tcp_link', 'world')
    newpose = movement_world * current_pose
    robot.lin(newpose)

    #robot lin movement nach unten rechts -> relative im world-KOS
    movement_world = Affine((0, 0.2, -0.2))
    current_pose = robot.node.get_transform('tcp_link', 'world')
    newpose = movement_world * current_pose
    robot.lin(newpose)

    #robot lin movement nach unten links -> relative im world-KOS
    movement_world = Affine((0, -0.2, 0))
    current_pose = robot.node.get_transform('tcp_link', 'world')
    newpose = movement_world * current_pose
    robot.lin(newpose)

    #robot lin movement nach oben links -> relative im world-KOS
    movement_world = Affine((0, 0, 0.2))
    current_pose = robot.node.get_transform('tcp_link', 'world')
    newpose = movement_world * current_pose
    robot.lin(newpose)

    #robot lin movement nach oben mitte-> relative im world-KOS
    movement_world = Affine((0, 0.1, 0.1))
    current_pose = robot.node.get_transform('tcp_link', 'world')
    newpose = movement_world * current_pose
    robot.lin(newpose)

    #robot lin movement nach oben rechts> relative im world-KOS
    movement_world = Affine((0, 0.15, -0.15))
    current_pose = robot.node.get_transform('tcp_link', 'world')
    newpose = movement_world * current_pose
    robot.lin(newpose)

    #robot lin movement nach unten rechts> relative im world-KOS
    movement_world = Affine((0, 0, -0.2))
    current_pose = robot.node.get_transform('tcp_link', 'world')
    newpose = movement_world * current_pose
    robot.lin(newpose)

    #robot move to home position (oben festgelegt)
    robot.home()


    # destroy the robot node
    robot.destroy_node()
    # shutdown previously initialized context
    rclpy.shutdown()
