#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from time import sleep
from std_msgs.msg import Bool

class Homologation(Node):
    '''
    This class is the node for the homologation.

    What the robot should do:
    - Start moving forward
    - Stop moving forward
    - Move the arm and the chain
    - Go to the final place
    - Start on the left, go to the right

    - If enemy is detected, stop the robot
    '''
    def __init__(self):
        super().__init__('homologation')
        self.get_logger().info('Homologation node has been started')

        # Make a publisher for /cmd_vel
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Create subscription to boolean /start
        # Change this to the topic of LA TIRETTE
        self.start_subscriber = self.create_subscription(Bool, 'start', self.start_callback, 10)

        self.get_logger().info('Homologation node has been started')
    def start_callback(self, msg):
        self.get_logger().info('Start message received')
        if msg.data:
            self.get_logger().info('Start message is True')
            self.stop()
        else:
            self.get_logger().info('Start message is False -> Stop the robot')
            self.stop()

    def stop(self):
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)
        self.get_logger().info('Robot should be stopped')


def main(args=None):
    rclpy.init(args=args)

    node = Homologation()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()