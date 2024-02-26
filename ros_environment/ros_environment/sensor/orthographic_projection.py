import struct
import ctypes
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
from ros_environment.lib.base_node import BaseNode


def get_heightmap(points, colors, bounds, pixel_size):
    width = int(np.round((bounds[0, 1] - bounds[0, 0]) / pixel_size))
    height = int(np.round((bounds[1, 1] - bounds[1, 0]) / pixel_size))
    heightmap = np.zeros((width, height), dtype=np.float32)
    colormap = np.zeros((width, height, colors.shape[-1]), dtype=np.uint8)

    # Filter out 3D points that are outside of the predefined bounds.
    ix = (points[Ellipsis, 0] >= bounds[0, 0]) & (points[Ellipsis, 0] < bounds[0, 1])
    iy = (points[Ellipsis, 1] >= bounds[1, 0]) & (points[Ellipsis, 1] < bounds[1, 1])
    iz = (points[Ellipsis, 2] >= bounds[2, 0]) & (points[Ellipsis, 2] < bounds[2, 1])
    valid = ix & iy & iz
    points = points[valid]
    colors = colors[valid]

    # Sort 3D points by z-value, which works with array assignment to simulate
    # z-buffering for rendering the heightmap image.
    iz = np.argsort(points[:, -1])
    sorted_points, sorted_colors = points[iz], colors[iz]
    p_indices_x = np.int32(np.round((sorted_points[:, 0] - bounds[0, 0]) / pixel_size))
    p_indices_y = np.int32(np.round((sorted_points[:, 1] - bounds[1, 0]) / pixel_size))
    p_indices_x = np.clip(p_indices_x, 0, width - 1)
    p_indices_y = np.clip(p_indices_y, 0, height - 1)
    heightmap[p_indices_x, p_indices_y] = (sorted_points[:, 2] - bounds[2, 0]) / (bounds[2, 1] - bounds[2, 0])
    for c in range(colors.shape[-1]):
        colormap[p_indices_x, p_indices_y, c] = sorted_colors[:, c]
    return heightmap, colormap


class OrthographicProjectionSensor:
    def __init__(self, pointcloud_topic: str, bounds: np.array, node: Node = None):
        if node is None:
            self.node = BaseNode("op_sensor")
        else:
            self.node = node

        self.bounds = bounds
        self.pc_sub = self.node.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pc_callback,
            10
        )

        self.observation = {}
        self.observation_valid = False
        self.current_transform = None
        self.observe = False

    def pc_callback(self, msg):
        if self.observe:
            self.observe = False
            xyz = []
            rgb = []
            gen = pc2.read_points(msg, skip_nans=True)
            int_data = list(gen)
            if self.current_transform is None:
                return
            for x in int_data:
                if 0 < x[2] < 2:
                    test = x[3]
                    s = struct.pack('>f', test)
                    i = struct.unpack('>l', s)[0]
                    pack = ctypes.c_uint32(i).value
                    r = (pack & 0x00FF0000) >> 16
                    g = (pack & 0x0000FF00) >> 8
                    b = (pack & 0x000000FF)
                    xyz.append([x[0], x[1], x[2], 1.0])
                    rgb.append([r, g, b])
            points = np.array(xyz)
            colors = np.array(rgb)
            t_points = (self.current_transform.matrix @ points.T).T[:, :3]
            h_map, c_map = get_heightmap(t_points, colors, self.bounds, 0.002)
            self.observation['color'] = c_map
            self.observation['depth'] = h_map
            self.observation_valid = True

    def get_observation(self):
        self.current_transform = self.node.get_transform()
        self.observe = True
        while not self.observation_valid:
            rclpy.spin_once(self.node)
        self.observation_valid = False
        return copy.deepcopy(self.observation)
