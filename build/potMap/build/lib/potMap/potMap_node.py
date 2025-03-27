import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class PotentialMap(Node):
    def __init__(self):
        super().__init__('potential_map')


        """Suscriptions"""
        self.laser = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        """Publishers"""

        """Variables"""

        self.ranges = []
        self.deltaAng = 0.0
        self.angles = []

        #Forces#
        self.Fx = 0.0
        self.Fy = 0.001
        

    def lidar_callback(self, msg):
        
        self.ranges = np.asarray(msg.ranges)
        self.ranges[np.isinf(self.ranges)] = 3.5

        self.deltaAng = msg.angle_increment
        self.angles = np.arange(msg.angle_min,msg.angle_max,self.deltaAng)

        for i, deg in enumerate (self.angles):
            if (self.ranges[i]<2.61):
                self.Fx = self.Fx + (1/self.ranges[i])**2 * np.cos(deg)
                self.Fy = self.Fy + (1/self.ranges[i])**2 * np.sin(deg)
        
        print("Fx",self.Fx)
        print("Fy",self.Fy)
  

def main(args=None):
    rclpy.init(args=args)
    PotMap = PotentialMap()
    rclpy.spin(PotMap)
    PotMap.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
