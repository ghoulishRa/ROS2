
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np

class puzzle_sim(Node):

    def __init__(self):
        super().__init__('puzzle_sim')

        #Publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)


        self.initial_pose_x = 0.0
        self.initial_pose_y = 0.0
        self.initial_pose_z = 0.0
        self.initial_pose_yaw = np.pi/2
        self.initial_pose_pitch = 0.0
        self.initial_pose_roll = 0.0


        #initialise Message to be published
        self.ctrlJoints = JointState()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["base_link_joint", "wheel_left_joint", "wheel_right_joint"]
        self.ctrlJoints.position = [0.0] * 3
        self.ctrlJoints.velocity = [0.0] * 3
        self.ctrlJoints.effort = [0.0] * 3

        #Create a Timer
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)


        #transform boradcaster#
        self.tf_br_base = TransformBroadcaster(self)
 
        #variables#

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.omega = 2.0
        #define transformation#

        self.define_TF()

    def timer_cb(self):

        time = self.get_clock().now().nanoseconds/1e9
        delta = 0.1

        # Create Trasnform Messages
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.transform.translation.x = self.initial_pose_x + 0.5*np.cos(self.omega*time)
        self.base_link_tf.transform.translation.y = self.initial_pose_y + 0.5*np.sin(self.omega*time)
        self.base_link_tf.transform.translation.z = self.initial_pose_z
        q = transforms3d.euler.euler2quat(self.initial_pose_roll, self.initial_pose_pitch, self.initial_pose_yaw+self.omega*time)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]

        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.position[1] += delta
        self.ctrlJoints.position[2] += delta

        self.tf_br_base.sendTransform(self.base_link_tf)
        self.publisher.publish(self.ctrlJoints)


    def define_TF(self):

        #Create Trasnform Messages (base_link)
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.header.frame_id = 'odom'
        self.base_link_tf.child_frame_id = 'base_link'
        self.base_link_tf.transform.translation.x = self.initial_pose_x
        self.base_link_tf.transform.translation.y = self.initial_pose_y
        self.base_link_tf.transform.translation.z = self.initial_pose_z
        q_foot = transforms3d.euler.euler2quat(self.initial_pose_roll, self.initial_pose_pitch, self.initial_pose_yaw)       
        self.base_link_tf.transform.rotation.x = q_foot[1]
        self.base_link_tf.transform.rotation.y = q_foot[2]
        self.base_link_tf.transform.rotation.z = q_foot[3]
        self.base_link_tf.transform.rotation.w = q_foot[0]


        
def main(args=None):
    rclpy.init(args=args)
    node = puzzle_sim()
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