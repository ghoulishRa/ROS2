import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import transforms3d


class URDFTransformPublisher(Node):
    def __init__(self):
        super().__init__('urdf_transform_publisher')

        self.static_broadcaster = StaticTransformBroadcaster(self)

        static_transforms = [
            self.create_static_transform('base_footprint', 'base_link', [0, 0, 0.05], [0, 0, 0]),
            self.create_static_transform('base_link', 'caster_holder_link', [-0.095, 0, -0.0307], [0, 0, 0]),
            self.create_static_transform('base_link', 'camera_base_link', [0.112, 0, 0.065], [0, 0, 0]),
            self.create_static_transform('base_link', 'lidar_base_link', [0.05, 0, 0.110], [0, 0, 0]),
        ]
        self.static_broadcaster.sendTransform(static_transforms)

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()

    def create_static_transform(self, parent, child, translation, rpy):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        q = transforms3d.euler.euler2quat(*rpy)
        t.transform.rotation.x = float(q[1])
        t.transform.rotation.y = float(q[2])
        t.transform.rotation.z = float(q[3])
        t.transform.rotation.w = float(q[0])
        return t


    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        angle = elapsed  # Simulacion de giro

        self.br.sendTransform(self.create_dynamic_transform(
            'base_link', 'wheel_right_link',
            [0.052, -0.095, -0.0025],
            [0, angle, 0]
        ))

        self.br.sendTransform(self.create_dynamic_transform(
            'base_link', 'wheel_left_link',
            [0.052, 0.095, -0.0025],
            [0, angle, 0]
        ))

    def create_dynamic_transform(self, parent, child, translation, rpy):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        q = transforms3d.euler.euler2quat(*rpy)
        t.transform.rotation.x = float(q[1])
        t.transform.rotation.y = float(q[2])
        t.transform.rotation.z = float(q[3])
        t.transform.rotation.w = float(q[0])
        return t



def main(args=None):
    rclpy.init(args=args)
    node = URDFTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()