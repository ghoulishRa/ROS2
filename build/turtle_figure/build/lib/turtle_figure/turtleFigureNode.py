import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import time



class Turtle_Figure(Node):
    def __init__(self):
        super().__init__('turtle_figure')

        """Publishers"""
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        """Variables"""

        self.speed = Twist()
    
        self.counter = 0

        timer_period = 0.1

        self.steps = 0

        """Timers"""
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counting_timer = self.create_timer(timer_period, self.counting_timer_callback)

    def move_turtle(self, linear_speed, angular_speed):
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)

    def counting_timer_callback(self):
        self.steps += 1
        print(self.steps)
    
    def draw_circle (self):
        self.speed.linear.x = 0.5
        self.speed.angular.z = 0.5

    def draw_square (self):
        self.speed.linear.x = 0.5
        self.speed.angular.z = 0.0
        time.sleep(1)

    def timer_callback(self):
        
        if self.counter == 0:
            self.draw_circle()
            if self.steps == 125 :
                self.counter = 2
        if self.counter %1  ==  1:
            self.draw_circle2()
        
        print(self.counter)

        self.publisher.publish(self.speed)




def main(args=None):
    rclpy.init(args=args)
    turtleFig = Turtle_Figure()
    rclpy.spin(turtleFig)
    turtleFig.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
