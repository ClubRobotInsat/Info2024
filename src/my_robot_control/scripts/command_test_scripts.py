#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Vector3
import threading
import numpy as np
from time import sleep

class MOTOR:
    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3


class Square(Node):

    def __init__(self):
        super().__init__("Square")
        self.joint = JointState()
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.joint_state_subscriber = self.create_subscription(JointState, "/joint_state", self.motor_control, 10)

        self.actions = [(200, 0), (0, 200), (-200, 0), (0, -200)]
        self.actions_index = 0

    def motor_control(self, joints: JointState):
        pos = ((joints.position[MOTOR.FRONT] + joints.position[MOTOR.BACK])/2, (joints.position[MOTOR.LEFT] + joints.position[MOTOR.RIGHT])/2)
        action = self.actions[self.actions_index]
        # (fw, lateral)
        if action[0] == 0 and action[1] == 0:
            self.get_logger().error(" unable to execute command in X and Y simultaneously")
            return

        if action[0] != 0 and pos[0] <= action[0]:
            v = Twist()
            v.linear.x = 0
            v.linear.z = 0
            v.linear.y = 0.05
            self.cmd_publisher.publish(v)
        elif action[1] != 0 and pos <= action[1]:
            v = Twist()
            v.linear.x = 0
            v.linear.z = 0
            v.linear.y = 0.05
            self.cmd_publisher.publish(v)
        else:
            v = Twist()
            v.linear.x = 0
            v.linear.z = 0
            v.linear.y = 0
            self.cmd_publisher.publish(v)
            self.actions_index += 1



def main(args=None):
    rclpy.init(args=args)
    sleep(2)
    node = Square()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


