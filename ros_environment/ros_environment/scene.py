# -*- coding: utf-8 -*-
import time

import copy
import rclpy
from tf2_ros import TransformException
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import numpy as np
from typing import List, Dict, Any, Tuple
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs_py.point_cloud2 as pc2
import struct
import ctypes
from manipulation_tasks.transform import Affine
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3

from .lib.base_node import BaseNode
from .robot import RobotClient
import manipulation_tasks.factory as factory
from ros_environment.sensor.camera import Camera, RGBDCamera
from ros_environment.sensor.orthographic_projection import OrthographicProjectionSensor
from .util import affine_to_pose

# from .sensor.camera import Camera


# TODO use manipulation_tasks Scene, Robot and Camera (Sensor) protocols
#  or maybe actually not Scene because of inapplicable functions


available_sensor_types = {
    'orthographic_projection': OrthographicProjectionSensor,
    'camera': Camera,
    'rgbd_cam': RGBDCamera
}


class RosScene(BaseNode):
    """
    TODO description
    """

    def __init__(self, sensor_config, is_simulation: bool = True):
        """
        TODO docstring

        Returns
        -------
        None.

        """
        # TODO namespaces!!!!
        super().__init__("ros_scene", is_simulation)
        # self.pixel_size = 0.002  # (155, 255, 3)
        # self.grasp_offset = Affine(translation=[0.0, 0, 0.602])
        self.robot = RobotClient(node=self, is_simulation=is_simulation)

        self.sensors = {}

        for key, config in sensor_config.items():
            config_dict = copy.deepcopy(config)
            sensor_type = config_dict.pop('type')
            self.sensors[key] = available_sensor_types[sensor_type](**config_dict, node=self)

        self.current_markers = []
        self.marker_publisher = self.create_publisher(MarkerArray, 'coordinate_frames', 10)

        # TODO get real values for resolution, ...
        # if resolution is None:
        #     self.resolution = (480, 640)
        # else:
        #     self.resolution = resolution
        # if intrinsics is None:
        #     self.intrinsics = (450., 0, 320., 0, 450., 240., 0, 0, 1)
        # else:
        #     self.intrinsics = intrinsics
        # self.depth_range = (0.01, 10.)
        # camera_poses = [Affine()]  # TODO change
        fixed = True
        # self.cameras = [
        #     Camera(camera_poses[0], self.resolution, self.intrinsics,
        #            self.depth_range, fixed=fixed)]

        # TODO how to get real t_bounds and r_bounds
        # if t_bounds is None:
        #     self.t_bounds = np.array([[0.25, 0.75], [-0.5, 0.5], [0, 0]])
        # else:
        #     self.t_bounds = t_bounds
        # self.r_bounds = np.array([[0, 0], [0, 0], [0, 2 * np.pi]])
        # self.bridge = CvBridge()

        # self.sensors = {
        #     'color_cam': {
        #         'type': Image,
        #         'topic': '/camera/color/image_raw',
        #         'callback': self.img_callback,
        #         'incoming_message_queue': 10,
        #         'link_name': 'camera_color_optical_frame',
        #         'pose': None,
        #         'intrinsics': intrinsics
        #     }
        # }
        # self.sensor_subs = {}
        # self.observe_sensor = {}
        # self.observations = {}
        #
        # for sensor_name in self.sensors.keys():
        #     self.sensor_subs[sensor_name] = self.create_subscription(
        #         self.sensors[sensor_name]['type'],
        #         self.sensors[sensor_name]['topic'],
        #         self.sensors[sensor_name]['callback'],
        #         self.sensors[sensor_name]['incoming_message_queue']
        #     )
        #     self.observe_sensor[sensor_name] = False

        # data for observation
        # self.observe = False
        # self.observation = {}
        #
        # self.current_transform = None
        # self.camera = None
        # # self.orthographic_projection = orthographic_projection
        #
        # # if self.orthographic_projection:
        # #     self.pc_sub = self.create_subscription(
        # #         PointCloud2,
        # #         '/d415/camera/pointcloud',
        # #         self.pc_callback,
        # #         10
        # #     )
        # # else:
        # self.img_sub = self.create_subscription(
        #     Image,
        #     '/d415/camera/color/image_raw',
        #     self.img_callback,
        #     10
        # )
        # cv bridge to convert ros images to numpy arrays

    # def img_callback(self, msg):
    #     sensor_name = 'color_cam'
    #     if self.observe_sensor[sensor_name]:
    #         try:
    #             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #         except CvBridgeError as e:
    #             print(e)
    #         self.observations[sensor_name] = {
    #             'color': cv_image,
    #             'intrinsics': self.sensors[sensor_name]['intrinsics'],
    #             'pose': self.sensors[sensor_name]['pose']
    #         }
    #         self.observe_sensor[sensor_name] = False

    # def store_sensor_pose(self, sensor_name: str):
    #     self.sensors[sensor_name]['pose'] = self.get_transform(to_frame_rel='cell_link',
    #                                                            from_frame_rel=self.sensors[sensor_name]['link_name'])

    def get_observation(self, sensor_name: str):
        if sensor_name not in self.sensors.keys():
            raise ValueError(f"Sensor {sensor_name} not available.")
        return self.sensors[sensor_name].get_observation()

    def spawn_coordinate_frame(self, pose: Affine):
        # TODO remove function after testing (not applicable to real scene)
        # TODO add content

        axis_marker = Marker()
        axis_marker.header.frame_id = 'world'
        axis_marker.header.stamp = self.get_clock().now().to_msg()
        axis_marker.scale = Vector3(x=0.1, y=0.01, z=0.01)
        axis_marker.color.a = 1.0
        axis_marker.type = 0
        axis_marker.action = 0

        marker_id = len(self.current_markers)
        x_marker = copy.deepcopy(axis_marker)
        x_marker.id = marker_id
        x_marker.pose = affine_to_pose(pose)
        x_marker.color.r = 1.0
        self.current_markers.append(marker_id)

        marker_id = len(self.current_markers)
        y_marker = copy.deepcopy(axis_marker)
        y_marker.id = marker_id
        y_pose = pose * Affine(rotation=[0, 0, np.pi / 2])
        y_marker.pose = affine_to_pose(y_pose)
        y_marker.color.g = 1.0
        self.current_markers.append(marker_id)

        marker_id = len(self.current_markers)
        z_marker = copy.deepcopy(axis_marker)
        z_marker.id = marker_id
        z_pose = pose * Affine(rotation=[0, -np.pi / 2, 0])
        z_marker.pose = affine_to_pose(z_pose)
        z_marker.color.b = 1.0
        self.current_markers.append(marker_id)

        marker_array = MarkerArray(markers=[x_marker, y_marker, z_marker])
        self.marker_publisher.publish(marker_array)

    def clean(self):
        # TODO remove function after testing (not applicable to real scene)
        # TODO add content
        markers = []
        for marker_id in self.current_markers:
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.id = marker_id
            marker.action = 2
            markers.append(marker)

        marker_array = MarkerArray(markers=markers)
        self.marker_publisher.publish(marker_array)

    def shutdown(self):
        self.destroy_node()

    # def get_current_pose(self):
    #     transform_tcp_link = self.get_transform('cell_link', 'tcp_link')
    #     curret_pose = PoseMsg()
    #     curret_pose.position.x = transform_tcp_link.translation[0]
    #     curret_pose.position.y = transform_tcp_link.translation[1]
    #     curret_pose.position.z = transform_tcp_link.translation[2]
    #     curret_pose.orientation.x = transform_tcp_link.quat[0]
    #     curret_pose.orientation.y = transform_tcp_link.quat[1]
    #     curret_pose.orientation.z = transform_tcp_link.quat[2]
    #     curret_pose.orientation.w = transform_tcp_link.quat[3]
    #     return RobotClient.pose_to_affine(curret_pose)

    # def get_current_pixel_pose(self):
    #     current_pose = self.get_current_pose()
    #
    #     u_v = position_to_pixel(current_pose.translation, self.t_bounds, self.pixel_size)
    #
    #     return u_v[0], u_v[1], current_pose.rpy[2]


# def position_to_pixel(position, bounds, pixel_size):
#     # TODO maybe clip
#     u = int(np.round((position[0] - bounds[0, 0]) / pixel_size))
#     v = int(np.round((position[1] - bounds[1, 0]) / pixel_size))
#     return u, v
#
#
# # TODO register RosScene (and find better name...)
#
# def get_heightmap(points, colors, bounds, pixel_size):
#     width = int(np.round((bounds[0, 1] - bounds[0, 0]) / pixel_size))
#     height = int(np.round((bounds[1, 1] - bounds[1, 0]) / pixel_size))
#     heightmap = np.zeros((width, height), dtype=np.float32)
#     colormap = np.zeros((width, height, colors.shape[-1]), dtype=np.uint8)
#
#     # Filter out 3D points that are outside of the predefined bounds.
#     ix = (points[Ellipsis, 0] >= bounds[0, 0]) & (points[Ellipsis, 0] < bounds[0, 1])
#     iy = (points[Ellipsis, 1] >= bounds[1, 0]) & (points[Ellipsis, 1] < bounds[1, 1])
#     iz = (points[Ellipsis, 2] >= bounds[2, 0]) & (points[Ellipsis, 2] < bounds[2, 1])
#     valid = ix & iy & iz
#     points = points[valid]
#     colors = colors[valid]
#
#     # Sort 3D points by z-value, which works with array assignment to simulate
#     # z-buffering for rendering the heightmap image.
#     iz = np.argsort(points[:, -1])
#     sorted_points, sorted_colors = points[iz], colors[iz]
#     p_indices_x = np.int32(np.round((sorted_points[:, 0] - bounds[0, 0]) / pixel_size))
#     p_indices_y = np.int32(np.round((sorted_points[:, 1] - bounds[1, 0]) / pixel_size))
#     p_indices_x = np.clip(p_indices_x, 0, width - 1)
#     p_indices_y = np.clip(p_indices_y, 0, height - 1)
#     heightmap[p_indices_x, p_indices_y] = (sorted_points[:, 2] - bounds[2, 0]) / (bounds[2, 1] - bounds[2, 0])
#     for c in range(colors.shape[-1]):
#         colormap[p_indices_x, p_indices_y, c] = sorted_colors[:, c]
#     return heightmap, colormap

def register() -> None:
    """Function to register the pybullet scene plugin"""
    # todo find out why protocol is not recognized -> check with mypy
    factory.register_simulated_scene('ros', RosScene)
