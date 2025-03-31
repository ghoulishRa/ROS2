import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
#   from nav_msgs.msg import Odometry
import tf2_ros
import transforms3d
import numpy as np


class PotentialMap(Node):
    def __init__(self):
        super().__init__('potential_map')


        """Suscriptions"""
        self.laser = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.click_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        """Publishers"""
        self.robot_speed = self.create_publisher(Twist, 'cmd_vel', 10)

        """Timers """

        self.tf_timer = self.create_timer(0.1, self.tf_timer_cb)
        self.speed_timer = self.create_timer(0.1, self.speed_timer_cb) 

        """Variables"""
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta_rob = 0.0

        self.click_x = -1.0
        self.click_y = 1.0
        #self.click_theta = 0.0

        """Messages"""
        self.speed = Twist()

        #Lidar data#
        self.ranges = []
        self.deltaAng = 0.0
        self.angles = []

        #Forces#
        self.F_rep_ang = 0.0
        self.F_rep_mag = 0.0  

        self.Fx_rep = 0.0
        self.Fy_rep = 0.0
  
        
        self.k_att = 4.0 #attraction force to clickpoint

        self.k_linear = 0.05
        self.k_angular  = 2.0


    def click_cb (self, msg):

        self.click_x = msg.point.x
        self.click_y = msg.point.y

        
        
    def tf_timer_cb (self):

        self.transformation = self.tf_buffer.lookup_transform(
            'odom',
            'base_link',
            rclpy.time.Time())

        orientation = self.transformation.transform.rotation

        q = [orientation.w, orientation.x, orientation.y, orientation.z]

        _, _, yaw = transforms3d.euler.quat2euler(q)

        self.pose_x = self.transformation.transform.translation.x
        self.pose_y = self.transformation.transform.translation.y
        self.theta_rob = yaw
        if self.theta_rob < 0.0:
            self.theta_rob += 2.0*np.pi
    
    def lidar_callback(self, msg):


        
        self.ranges = np.asarray(msg.ranges)
        self.ranges[np.isinf(self.ranges)] = 3.5

        self.deltaAng = msg.angle_increment
        self.angles = np.arange(msg.angle_min,msg.angle_max,self.deltaAng)


        for i, deg in enumerate (self.angles):
            if (self.ranges[i]<0.5):
                self.Fx_rep = self.Fx_rep + (1/self.ranges[i])**2 * np.cos(deg)
                self.Fy_rep = self.Fy_rep + (1/self.ranges[i])**2 * np.sin(deg)




        #getting repulsion force magnitude and angle#
        self.F_rep_ang = np.arctan2(self.Fy_rep, self.Fx_rep) - self.theta_rob #add pi to invert direction
        self.F_rep_mag = np.linalg.norm((self.Fx_rep, self.Fy_rep)) 

        # self.speed.linear.x = self.k_linear * self.F_rep_mag
        # self.speed.angular.z = self.k_angular * self.F_rep_ang

        # print("linear_speed:", self.speed.linear.x)
        # print("angular_speed:", self.speed.angular.z)

        #self.robot_speed.publish(self.speed)

    def speed_timer_cb (self):

            dx = self.pose_x - self.click_x
            dy = self.pose_y - self.click_y

            Fx_att = self.k_att * dx
            Fy_att = self.k_att * dy

            print("Fx_att: ", Fx_att)
            print("Fy_att: ", Fy_att)

            print("Fx_rep: ", self.Fx_rep)
            print("Fy_rep: ", self.Fy_rep)

            print("yaw:", self.theta_rob)

            d_goal = np.sqrt(dx**2 + dy**2)

            # F_att_ang  = np.arctan2(Fy_att,Fx_att)
            # F_att_mag = np.linalg.norm((Fx_att,Fy_att))

            Fx_total = Fx_att - self.Fx_rep
            Fy_total = Fy_att - self.Fy_rep

            F_total_ang = (np.arctan2(Fy_total,Fx_total) + np.pi) - self.theta_rob
            F_total_mag = np.sqrt(Fx_total**2 + Fy_total**2)

            print("F_total_ang", F_total_ang )
            print("F_total_mag", F_total_mag )

            self.speed.linear.x = self.k_linear * F_total_mag
            self.speed.angular.z = -(self.k_angular * F_total_ang)

            if d_goal < 0.1:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0

            print ("click_x:",self.click_x)
            print ("click_y:",self.click_y)

            print("linear_speed:", self.speed.linear.x)
            print("angular_speed:", self.speed.angular.z)

            self.robot_speed.publish(self.speed)


def main(args=None):
    rclpy.init(args=args)
    PotMap = PotentialMap()
    rclpy.spin(PotMap)
    PotMap.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()