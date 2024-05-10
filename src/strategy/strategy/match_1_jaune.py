#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from time import sleep
from std_msgs.msg import Bool
from can_interface.msg import ArmFeedback, Tirette, Enemy
from sensor_msgs.msg import JointState


class MatchJaune(Node):
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

        self.tirette_subscriber = self.create_subscription(Tirette, 'tirette', self.start_callback, 10)
        self.motor_subscriber = self.create_subscription(JointState, 'motors_feedback', self.motor_callback, 10)

        self.enemy_subscriber = self.create_subscription(Enemy, 'enemy', self.enemy_callback, 10)
        self.get_logger().info('Homologation node initialized')

        self.allowed_moving = True
        self.move_timer = None

        self.sequence_index = 0
        self.instructions = [
            # (0.0, 0.15, 0.5),
            (0.0, 0.30, 1.75),  # Go front 52.5 cm
            (0.0, 0.0, 0.2),    # STOP
            (-0.30, 0.0, 5.0),  # Go left 165 cm
            # Move arm and chain
            # (0.0, 0.0, 0.5),

            # Go to final area
            # (0.30, 0.0, 3.0),
            (0.0, 0.0, 0.2),    # STOP
            (0.0, -0.30, 1.75),  # GO BACK 52.5 cm

            # Go the the wall on the left
            # Then move a bit onto the right
            # Go forward
            # Go a bit onto the right
            # Go backward
            (-0.20,0.0, 1.0), # Left to the wall
            (0.0, 0.0, 0.2),    # STOP SHOULD BE AT THE CORNER
            (0.30, 0.30, 0.4), # Diagonal to leave the corner
            (0.0, 0.20, 4.0), # Forward to swap solar panels
            (0.30, 0.0, 0.2), # A bit to the right
            (0.0, -0.30, 2.5),
        ]

    def execute_next_instruction(self):
        if self.sequence_index < len(self.instructions):
            if self.move_timer is not None:
                self.move_timer.destroy()
            if self.allowed_moving:
                self.get_logger().info('Starting next instruction')
                instruction = self.instructions[self.sequence_index]
                self.move(*instruction)
                self.sequence_index += 1

        else:
            self.stop()
            self.get_logger().info('All instructions executed')
            self.move_timer.destroy()

    def start_callback(self, msg):
        self.get_logger().info('Start message received')
        if msg.go:
            self.get_logger().info('Start message is True')
            self.sequence_index = 0
            self.allowed_moving = True

            self.execute_next_instruction() # Recursive call -> call at the end of this scope
        else:
            self.get_logger().info('Start message is False -> Stop the robot')
            self.stop()

    def move_timer_callback(self):
        self.stop()
        if self.move_timer is not None:
            self.move_timer.destroy()

    def move(self, speed_x=0.0, speed_y=0.0, duration=1.0):
        # Move the robot
        twist = Twist()
        twist.linear.x = speed_x
        twist.linear.y = speed_y
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)
        self.get_logger().info(f"Robot should be moving at speed: x={speed_x} ; y={speed_y} for {duration} s.")

        self.move_timer = self.create_timer(duration, lambda: self.execute_next_instruction())


    def motor_callback(self,msg):
        pass

    def stop(self):
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)
        self.get_logger().info('Robot should be stopped')

    def enemy_callback(self, msg):
        self.allowed_moving = False
        self.stop()

def main(args=None):
    rclpy.init(args=args)

    node = MatchJaune()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
