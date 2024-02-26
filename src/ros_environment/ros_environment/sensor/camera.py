import copy
from typing import Tuple, Dict, Union

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from manipulation_tasks.transform import Affine

from ros_environment.lib.base_node import BaseNode


# TODO use manipulation_tasks.Camera


class Camera(Node):
    """
    TODO description
    """

    def __init__(
            self,
            topic: str = None,
            frame_id: str = None,
            pose: Affine = None,
            intrinsics: Tuple[float, float, float, float, float, float, float, float, float] = None,
            node: Node = None) -> None:
        if node is None:
            self.node = BaseNode("camera")
        else:
            self.node = node
        self.topic = topic
        self.pose = pose
        self.frame_id = frame_id
        self.intrinsics = intrinsics

        self.observation_valid = False
        self.image = None
        self.observe = False

        self.bridge = CvBridge()

        self.img_sub = self.node.create_subscription(
            Image,
            self.topic,
            self.img_callback,
            10
        )

    def img_callback(self, msg):
        if self.observe:
            self.observe = False
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, '8UC3')
            except CvBridgeError as e:
                return
            self.image = cv_image
            self.observation_valid = True

    def get_config(self):
        config = {
            "topic": self.topic,
            "pose": self.pose,
            "frame_id": self.frame_id,
            "intrinsics": self.intrinsics}
        return config

    def get_observation(self):
        if self.frame_id is not None:
            pose = self.node.get_transform(self.frame_id, 'world')
        else:
            pose = copy.deepcopy(self.pose)

        self.observe = True
        while not self.observation_valid:
            rclpy.spin_once(self.node)
        self.observation_valid = False
        observation = {
            'color': copy.deepcopy(self.image),
            'intrinsics': self.intrinsics,
            'pose': pose
        }
        return observation

    def read_data(self):
        # TODO no need to implement function as soon as manipulation_tasks.Camera inherited
        return self.get_observation()


class RGBDCamera(Node):
    """
    TODO description
    """

    def __init__(
            self,
            rgb_topic: str = None,
            depth_topic: str = None,
            frame_id: str = None,
            pose: Affine = None,
            intrinsics: Tuple[float, float, float, float, float, float, float, float, float] = None,
            node: Node = None) -> None:
        if node is None:
            self.node = BaseNode("camera")
        else:
            self.node = node
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.pose = pose
        self.frame_id = frame_id
        self.intrinsics = intrinsics

        self.observe_rgb = False
        self.rgb_observation_valid = False
        self.observe_depth = False
        self.depth_observation_valid = False
        self.rgb_image = None
        self.depth_image = None

        self.bridge = CvBridge()

        self.rgb_sub = self.node.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )

        self.depth_sub = self.node.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

    def rgb_callback(self, msg):
        if self.observe_rgb:
            self.observe_rgb = False
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, '8UC3')
            except CvBridgeError as e:
                return
            self.rgb_image = cv_image
            self.rgb_observation_valid = True

    def depth_callback(self, msg):
        if self.observe_depth:
            self.observe_depth = False
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg)
            except CvBridgeError as e:
                return
            self.depth_image = cv_image
            self.depth_observation_valid = True

    def get_config(self):
        config = {
            "topic": self.rgb_topic,
            "pose": self.pose,
            "frame_id": self.frame_id,
            "intrinsics": self.intrinsics}
        return config

    def get_observation(self):
        if self.frame_id is not None:
            pose = self.node.get_transform(self.frame_id, 'world')
        else:
            pose = copy.deepcopy(self.pose)

        self.observe_rgb = True
        self.observe_depth = True
        while not self.depth_observation_valid:
            rclpy.spin_once(self.node)
        self.depth_observation_valid = False
        observation = {
            'color': copy.deepcopy(self.rgb_image),
            'depth': copy.deepcopy(self.depth_image),
            'intrinsics': self.intrinsics,
            'pose': pose
        }
        return observation

    def read_data(self):
        # TODO no need to implement function as soon as manipulation_tasks.Camera inherited
        return self.get_observation()
