import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class turleFigureNode (Node):
    def __init__(self):
        super().__init__('turtle_figure')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        

        self.draw_all_shapes()

    def draw_square(self):
        twist = Twist()
        side_length = 2.0  # Time in seconds to move forward
        turn_duration = 1.0  # Time in seconds to turn 90 degrees

        for _ in range(4):
            # Move forward
            twist.linear.x = 2.0  # Speed
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(side_length)

            # Stop moving forward
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            time.sleep(0.5)

            # Turn 90 degrees
            twist.angular.z = 1.57  # Approx. 90 degrees per second
            self.publisher_.publish(twist)
            time.sleep(turn_duration)

            # Stop turning
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(0.5)

        #self.get_logger().info("Finished drawing a square.")
    
    def draw_triangle(self):
        twist = Twist()
        side_length = 2.0  # Time in seconds to move forward
        turn_duration = 1.2  # Approximate time to turn 120 degrees

        for _ in range(3):
            # Move forward
            twist.linear.x = 2.0  # Speed
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(side_length)

            # Stop moving forward
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            time.sleep(0.5)

            # Turn 120 degrees
            twist.angular.z = 2.09  # Approx. 120 degrees per second
            self.publisher_.publish(twist)
            time.sleep(turn_duration)

            # Stop turning
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(0.5)

        #self.get_logger().info("Finished drawing a triangle.")
    
    def draw_circle(self):
        twist = Twist()
        twist.linear.x = 2.0  # Forward speed
        twist.angular.z = 1.0  # Rotation speed (controls circle radius)
 
        # Publish the velocity continuously for smooth movement
        start_time = time.time()
        while time.time() - start_time < 6:  # Draw for 10 seconds
            self.publisher_.publish(twist)
            time.sleep(0.1)

        # Stop the turtle
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        
        #self.get_logger().info("Finished drawing a circle.")

    def draw_all_shapes(self):
        self.steps = 0
        self.state = 'IDLE'

        #create a dictionary to save states#
        state_actions = {
            'IDLE': self.draw_circle,
            'SQUARE': self.draw_square,
            'TRIANGLE': self.draw_triangle
        }

        while True:
            
            action = state_actions.get(self.state, None)

            #if the state was found, an actions runs and automatically add one step#
            if action:
                action()
                self.steps += 1

            match self.state:
                case 'IDLE':
                    self.state = 'TRIANGLE' if self.steps % 2 == 0 else 'SQUARE'
                case 'SQUARE':
                    self.state = 'TRIANGLE' if self.steps % 2 == 1 else 'SQUARE'
                case 'TRIANGLE':
                    self.state = 'CIRCLE' if self.steps % 2 == 0 else 'SQUARE'
                case 'CIRCLE':
                    self.state = 'IDLE' 

            # print("Counter:", self.steps)
            # print("State:", self.state)

def main(args=None):
    rclpy.init(args=args)
    TurtleFigure = turleFigureNode()
    rclpy.spin(TurtleFigure)
    TurtleFigure.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

