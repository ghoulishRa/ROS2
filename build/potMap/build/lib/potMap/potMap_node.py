import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, Pose2D, PoseStamped
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
        self.vector = self.create_publisher(PoseStamped, '/rob_vector', 10)

        """Timers """

        self.tf_timer = self.create_timer(0.1, self.tf_timer_cb)
        self.speed_timer = self.create_timer(0.02, self.speed_timer_cb) 

        """Variables"""
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.theta_rob = 0.0

        self.click = PointStamped()    #self.click_theta = 0.0

        self.waypoints = [[0.0, 2.0], [2.0, 0.0], [0.0, -2.0], [-2.0, 0.0]]

        self.click_x = self.waypoints[0][0]
        self.click_y = self.waypoints[0][1]

        self.d_goal = 0.0
        self.counter = 0

        """Messages"""
        self.speed = Twist()
        self.rob_pose = PoseStamped()

        #Lidar data#
        self.ranges = []
        self.deltaAng = 0.0
        self.angles = []

        #Forces#
        self.F_rep_ang = 0.0
        self.F_rep_mag = 0.0  

        self.Fx_rep = 0.0
        self.Fy_rep = 0.0
  
        
        self.k_att = 2.0 #attraction force to clickpoint
        self.k_rep = 1.0

        self.k_linear = 0.03
        self.k_angular  = 1.0


    def click_cb (self, msg):

        """getting the clicked_point pose"""

        self.click = msg

        self.click_x = self.click.point.x
        self.click_y = self.click.point.y
        
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
        self.Fx_rep = 0.0
        self.Fy_rep = 0.0

        self.ranges = np.asarray(msg.ranges)
        self.ranges[np.isinf(self.ranges)] = 3.5

        self.deltaAng = msg.angle_increment
        self.angles = np.arange(msg.angle_min,msg.angle_max,self.deltaAng)
        

        for i, deg in enumerate (self.angles):
            # if (0 <= deg <= (np.pi)/4) and (2*(np.pi)-(np.pi)/4 <= deg <= 2*(np.pi)):
            if (self.ranges[i]<0.45):
                self.Fx_rep += (1/self.ranges[i]) * np.cos(deg)
                self.Fy_rep += (1/self.ranges[i]) * np.sin(deg)

        self.Fx_rep = (self.Fx_rep/600.0)*40.0
        self.Fy_rep = (self.Fy_rep/600.0)*40.0


    def speed_timer_cb (self):

            """Repulsion forces calculation"""
            #getting repulsion force magnitude and angle#

            dx = self.pose_x - self.click_x
            dy = self.pose_y - self.click_y

            Fx_att = self.k_att * dx
            Fy_att = self.k_att * dy

            self.d_goal = np.sqrt(dx**2 + dy**2)

            Fx_total = Fx_att + (self.k_rep*self.Fx_rep)
            Fy_total = Fy_att + (self.k_rep*self.Fy_rep)

            F_total_ang = (np.arctan2(Fy_total,Fx_total) + np.pi) - self.theta_rob
            F_total_ang = np.arctan2(np.sin(F_total_ang), np.cos(F_total_ang))
            F_total_mag = np.sqrt(Fx_total**2 + Fy_total**2)

            self.speed.linear.x = self.k_linear * F_total_mag
            self.speed.angular.z = (self.k_angular * F_total_ang)

            self.state_machine()
            
            self.robot_speed.publish(self.speed)
            
    
    def state_machine (self):
        if self.d_goal < 0.1:
            print("entre a state machine")
            self.counter += 1
            self.click_x = self.waypoints[self.counter][0]
            self.click_y = self.waypoints[self.counter][1]
            if self.counter == len(self.waypoints)-1 :
                self.speed.angular.z = 0.0
                self.speed.linear.x = 0.0
                self.counter = 0


def main(args=None):
    rclpy.init(args=args)
    PotMap = PotentialMap()
    rclpy.spin(PotMap)
    PotMap.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()