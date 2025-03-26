import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PotentialMap(Node):
    def __init__(self):
        super().__init__('potential map')


        """Suscriptions"""
        self.laser = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        """Publishers"""

        """Variables"""

    def lidar_callback(self, msg):

        laser = msg 


def main(args=None):
    rclpy.init(args=args)
    PotentialMap = PotentialMap()
    rclpy.spin(PotentialMap)
    PotentialMap.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
