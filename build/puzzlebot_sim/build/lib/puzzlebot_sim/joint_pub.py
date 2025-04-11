
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class joint_pub(Node):

    def __init__(self):
        super().__init__('joint_pub')

        #Publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        #Create a Timer
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        #initialise Message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["base_link_joint", "wheel_left_joint", "wheel_right_joint"]
        self.ctrlJoints.position = [0.0] * 3
        self.ctrlJoints.velocity = [0.0] * 3
        self.ctrlJoints.effort = [0.0] * 3


    #Timer Callback
    def timer_cb(self):

        time = self.get_clock().now().nanoseconds/1e9

        delta = 0.1

        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.velocity[0] = 0.0
        self.ctrlJoints.velocity[1] = 0.5
        self.ctrlJoints.velocity[2] = 0.5
        
        self.ctrlJoints.position[1] += delta  # left
        self.ctrlJoints.position[2] += delta  # right
        
        self.publisher.publish(self.ctrlJoints)

def main(args=None):
    rclpy.init(args=args)
    node = joint_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()