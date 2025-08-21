import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TurtleMovementPublisher(Node):
    def __init__(self):
        super().__init__('turtle_movement_publisher')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cmd = Twist()

    def move_robot(self, movements):
        for move in movements:
            self.cmd = Twist()
            if 'forward' in move:
                self.cmd.linear.x = 1.0
            elif 'backward' in move:
                self.cmd.linear.x = -1.0
            elif 'left' in move:
                self.cmd.angular.z = 1.5
            elif 'right' in move:
                self.cmd.angular.z = -1.5
            elif 'stop' in move:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
            else:
                continue

            self.publisher.publish(self.cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(1)  # Simulate execution time

