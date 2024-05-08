#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from can_interface.msg import Enemy
import math

class EnemyDetection(Node):
    def __init__(self):
        super().__init__('enemy_detection')

        self.subscriber_laser_scan = self.create_subscription(LaserScan, 'scan', self.callback_laser_scan, 10)

        # Our orobt
        self.robot_radius = 0.4
        self.robot_minimal_detection = self.robot_radius / 2

        # Their robot
        self.enemy_radius = 0.4

        self.alert_detection = self.robot_minimal_detection + self.enemy_radius / 2 + 0.3

        self.enemy_publisher = self.create_publisher(Enemy, 'enemy', 10)

    def callback_laser_scan(self, msg):
        # self.get_logger().info('Received message on /scan: {0}'.format(msg))

        for i in range(len(msg.ranges)):
            current_angle = msg.angle_min + i * msg.angle_increment
            current_range = msg.ranges[i]

            if self.robot_minimal_detection <= current_range <= self.alert_detection:
                self.get_logger().info('Alert! Enemy detected at angle {0} and distance {1}'.format(math.degrees(current_angle), current_range))
                Enemy_msg = Enemy()
                Enemy_msg.angle = current_angle
                Enemy_msg.distance = current_range
                self.enemy_publisher.publish(Enemy_msg)

def main(args=None):
    rclpy.init(args=args)

    enemy_detection = EnemyDetection()

    rclpy.spin(enemy_detection)

    rclpy.shutdown()

if __name__ == '__main__':
    main()