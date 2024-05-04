#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from time import sleep
from std_msgs.msg import Bool

class Strategy(Node):
    def __init__(self):
        super().__init__('strategy')
        self.get_logger().info('Strategy node has been started')

        # Make a publisher for /cmd_vel
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Create subscription to boolean /start
        self.subscription = self.create_subscription(Bool, 'start', self.test, 10)

        self.timer = None

    def timer_callback(self):
        self.stop()
        self.timer.destroy()

    def move_forward(self, x=0.0, y=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = x / duration
        twist.linear.y = y / duration
        self.publisher_.publish(twist)
        self.get_logger().info('Robot is moving forward')

        self.timer = self.create_timer(duration, self.timer_callback)


    def turn_degrees(self, degrees, duration=1.0):
        twist = Twist()
        twist.angular.z = 2 * math.radians(degrees) / duration
        self.publisher_.publish(twist)
        self.get_logger().info('Robot is turning ' + str(degrees) + ' degrees')

        self.timer = self.create_timer(duration, self.timer_callback)

    def stop(self):
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info('Robot has stopped')

    def go_to_pot_place(self):
        # Move 5 cm on y axis
        # Then move 75 cm on -x axis
        self.move_forward(0.0,0.05, duration=1.0)
        self.move_forward(-0.25, duration=3.0)

    def go_to_plant_place(self):
        # Move 1 meter on y axis
        dur = 3.0
        dist = 1.0
        self.move_forward(0.0, dist / dur, duration=dur)
        self.stop()
    def test(self, data):
        # Turn the robot 360 degrees
        # self.turn_degrees(360, duration=5.0)
        if data:
            # self.move_forward(0.0, 1.0, duration=5.0)
            self.turn_degrees(360, duration=5.0)
            # self.go_to_plant_place()
        else:
            self.stop()




def main(args=None):
    rclpy.init(args=args)

    node = Strategy()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()