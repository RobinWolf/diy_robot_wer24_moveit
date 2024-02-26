import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from manipulation_tasks.transform import Affine # import the affine trafo libary from dependencies (only valid in docker container)


class BaseNode(Node):
    def __init__(self, name, simulation=True):
        super().__init__(name)
        self.simulation = simulation

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self, from_frame_rel, to_frame_rel='world'):
        counter = 0
        has_transform = False
        transform = None
        while (not has_transform) and counter < 10:
            rclpy.spin_once(self)
            counter += 1
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                    to_frame_rel, from_frame_rel, now)
                trans = trans.transform
                transform = Affine(
                    [trans.translation.x, trans.translation.y, trans.translation.z],
                    [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])
                has_transform = self.tf_buffer.can_transform(
                    to_frame_rel, from_frame_rel, now)
                # print("Has transform: ", has_transform, counter)
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return transform
