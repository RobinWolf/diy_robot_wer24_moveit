# -*- coding: utf-8 -*-
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from moveit_wrapper.srv import MoveToPose, MoveToJointPosition, String  #import the custom service interfaces from the wrapper package
from geometry_msgs.msg import Pose as PoseMsg
from manipulation_tasks.transform import Affine
from geometry_msgs.msg import Quaternion, Point
from std_srvs.srv import SetBool, Trigger    #change in SetBool for our gripper driver
import copy

from ros_environment.lib.base_node import BaseNode  #import the get_transform method to get affine transforms between X and world

from .util import affine_to_pose


# TODO use manipulation_tasks protocol for Robot


class RobotClient:  #this is the class which gets called in your application// when calling the init method the clients for the services in moveit_wrapper_node get initialized
    """ 
    TODO description
    """

    def __init__(self, node: Node = None, is_simulation: bool = False) -> None:
        """
        TODO docstring

        Returns
        -------
        None.

        """
        if node is None:
            self.node = BaseNode("robot_client", is_simulation) #starts the BaseNode (lib folder) --> get_transform method
        else:
            self.node = node

        self.move_lin_cli = self.node.create_client(MoveToPose, "/move_to_pose_lin")    #connects to the services defined in the moveit_wrapper srvs
        while not self.move_lin_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move_to_pose_lin service not available, waiting some more ...")
        self.node.get_logger().info("move_to_pose_lin service available")

        self.move_ptp_cli = self.node.create_client(MoveToPose, "/move_to_pose_ptp")
        while not self.move_ptp_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move_to_pose_ptp service not available, waiting some more ...")
        self.node.get_logger().info("move_to_pose_ptp service available")

        self.move_joint_cli = self.node.create_client(MoveToJointPosition, "/move_to_joint_position")
        while not self.move_joint_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("move_to_joint_position service not available, waiting some more ...")
        self.node.get_logger().info("move_to_joint_position service available")

        self.reset_planning_group_cli = self.node.create_client(String, "/reset_planning_group")
        while not self.reset_planning_group_cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("reset_planning_group service not available, waiting some more ...")
        self.node.get_logger().info("reset_planning_group service available")

        self.is_simulation = is_simulation  # set to true until the gripper driver works so the server node will not called
        if not self.is_simulation:
            self.gripper_cli = self.node.create_client(SetBool, "/gripper_control")       #bool service 0 open/ 1 close instead of 2 diffrent trigger services
            while not self.open_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("gripper_controller service not available, waiting again...")

        
        # TODO where to get home pose from
        self.home_position = [0.0,0.0,0.0,0.0,0.0,0.0]


    # this are the methods which can be called in your application which are communicating to the robot
    def home(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        None.

        """
        return self.ptp_joint(self.home_position)  # and gripper_success

    def ptp(self, pose: Affine) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.

        """
        req = MoveToPose.Request()
        req.pose = affine_to_pose(pose)
        future = RobotClient.send_request(req, self.move_ptp_cli)
        response = self.wait_for_response(future)
        return response.success

    def ptp_joint(self, joint_positions: List[float]) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.
        :param joint_positions:

        """
        req = MoveToJointPosition.Request()
        req.joint_position = joint_positions
        future = RobotClient.send_request(req, self.move_joint_cli)
        response = self.wait_for_response(future)
        return response.success

    def lin(self, pose: Affine) -> bool:
        """
        TODO docstring

        Parameters
        ----------
        pose : Affine
            DESCRIPTION.

        Returns
        -------
        None.

        """
        req = MoveToPose.Request()
        req.pose = affine_to_pose(pose)
        future = RobotClient.send_request(req, self.move_lin_cli)
        response = self.wait_for_response(future)
        return response.success

    def reset_planning_group(self, planning_group) -> bool:
        req = String.Request()
        req.data = planning_group
        future = RobotClient.send_request(req, self.reset_planning_group_cli)
        response = self.wait_for_response(future)
        return response.success

    def toggle_gripper(self, request = 0) -> bool:    #pass 0 for open and 1 for close
        """
        TODO docstring

        Returns
        -------
        bool
            DESCRIPTION.

        """
        s = True
        if not self.is_simulation:
            req = SetBool.Request() # init the message object
            req.data = request  #pass the given argument to the server node
            future = RobotClient.send_request(req, self.gripper_cli)
            response = self.wait_for_response(future)
            s = response.success
        if not s:
            self.node.get_logger().info("Opening gripper unsuccessful.")
        return s


    @staticmethod
    def send_request(request, client):
        future = client.call_async(request)
        return future

    def wait_for_response(self, future):
        """
        TODO docstring

        Parameters
        ----------
        future : TYPE
            DESCRIPTION.

        Returns
        -------
        response : TYPE
            DESCRIPTION.

        """
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                    return None
                else:
                    return response



    def destroy_node(self) -> None:
        self.node.destroy_node()

    def move_gripper(self, absolute_position=None, relative_position=None):
        pass
