# -*- coding: utf-8 -*-
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from moveit_wrapper.srv import MoveToPose, MoveToJointPosition, String
from geometry_msgs.msg import Pose as PoseMsg
from manipulation_tasks.transform import Affine
from geometry_msgs.msg import Quaternion, Point
from std_srvs.srv import Trigger    #change in SetBool for our gripper driver
import copy

from ros_environment.lib.base_node import BaseNode

from .util import affine_to_pose


# TODO use manipulation_tasks protocol for Robot


class RobotClient:
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
            self.node = BaseNode("robot_client", is_simulation)
        else:
            self.node = node
        self.move_lin_cli = self.node.create_client(MoveToPose, "/move_to_pose_lin")    #gegenstücke vom wrapper
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

        self.is_simulation = is_simulation  # so lange kein richtiges gripper interface da ist
        if not self.is_simulation:
            self.open_cli = self.node.create_client(Trigger, "/open_gripper")
            while not self.open_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("open_gripper service not available, waiting again...")
            self.close_cli = self.node.create_client(Trigger, "/close_gripper")
            while not self.close_cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("close_gripper service not available, waiting again...")
        # TODO where to get home pose from
        self.home_position = [np.pi / 2, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2]

        self.start_servo_client = self.node.create_client(
            Trigger, "servo_node/start_servo")

        self.stop_servo_client = self.node.create_client(
            Trigger, "servo_node/stop_servo")

        while not self.start_servo_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('start_servo_client not available, waiting again...')

        while not self.stop_servo_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('stop_servo_client not available, waiting again...')

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

    def open_gripper(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        bool
            DESCRIPTION.

        """
        s = True
        if not self.is_simulation:
            req = Trigger.Request() ## change to our Service oder Standard set_bool req.data = true für 1 = close
            future = RobotClient.send_request(req, self.open_cli)
            response = self.wait_for_response(future)
            s = response.success
        if not s:
            self.node.get_logger().info("Opening gripper unsuccessful.")
        return s

    def close_gripper(self) -> bool:
        """
        TODO docstring

        Returns
        -------
        bool
            DESCRIPTION.

        """
        s = True
        if not self.is_simulation:
            req = Trigger.Request()
            future = RobotClient.send_request(req, self.close_cli)
            response = self.wait_for_response(future)
            s = response.success
        if not s:
            self.node.get_logger().info("Closing gripper unsuccessful.")
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

    def enable_servo(self) -> bool:
        trigger = Trigger.Request()
        future = self.start_servo_client.call_async(trigger)
        response = self.wait_for_response(future)
        return response

    def disable_servo(self) -> bool:
        trigger = Trigger.Request()
        future = self.stop_servo_client.call_async(trigger)
        response = self.wait_for_response(future)
        return response

    def destroy_node(self) -> None:
        self.node.destroy_node()

    def move_gripper(self, absolute_position=None, relative_position=None):
        pass
