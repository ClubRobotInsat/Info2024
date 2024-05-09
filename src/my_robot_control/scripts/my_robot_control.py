#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
import numpy as np

axes = np.array([0,0,0], float)

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.wheel_vel = np.array([0,0,0,0], float)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        timer_period = 0.005
        # self.L = 0.1241 # SIMU distance from the robot center to wheel
        self.L = 0.135 # REAL distance from the robot center to wheel
        self.Rw = 0.041 # Radius ot the wheel

        self.vel_msg = Twist()
        self.threshold = 0.08
        self.linear_vel = 1.0 # m/s # Goal: 0.4m/s, going 0.9m/s
        self.omega = 1.0 # rad/s

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def timer_callback(self):
        global axes

        vel_x = axes[0]*self.linear_vel
        vel_y = axes[1]*self.linear_vel
        vel_w = axes[2]*self.omega

        # Left 3, Front 0, Right 1, Back 2
        # self.wheel_vel[3] = (vel_x*math.sin(math.pi/4            ) + vel_y*math.cos(math.pi/4            ) + self.L*vel_w)/self.Rw
        # self.wheel_vel[0] = (vel_x*math.sin(math.pi/4 + math.pi/2) + vel_y*math.cos(math.pi/4 + math.pi/2) + self.L*vel_w)/self.Rw
        # self.wheel_vel[1] = (vel_x*math.sin(math.pi/4 - math.pi)   + vel_y*math.cos(math.pi/4 - math.pi)   + self.L*vel_w)/self.Rw
        # self.wheel_vel[2] = (vel_x*math.sin(math.pi/4 - math.pi/2) + vel_y*math.cos(math.pi/4 - math.pi/2) + self.L*vel_w)/self.Rw
        #

        self.wheel_vel[3] = (vel_x*math.sin(0          ) + vel_y*math.cos(0          ) + self.L*vel_w)/self.Rw
        self.wheel_vel[0] = (vel_x*math.sin(+ math.pi/2) + vel_y*math.cos(+ math.pi/2) + self.L*vel_w)/self.Rw
        self.wheel_vel[1] = (vel_x*math.sin(- math.pi)   + vel_y*math.cos(- math.pi)   + self.L*vel_w)/self.Rw
        self.wheel_vel[2] = (vel_x*math.sin(- math.pi/2) + vel_y*math.cos(- math.pi/2) + self.L*vel_w)/self.Rw


        array_forPublish = Float64MultiArray(data=self.wheel_vel)
        # rclpy.logging._root_logger.info(f"wheel vel : {self.wheel_vel}")
        self.publisher_.publish(array_forPublish)

        # Compute odometry
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9 # in seconds




class CmdVel_subscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, data):
        global axes

        axes[0] = data.linear.x
        axes[1] = data.linear.y
        axes[2] = data.angular.z


if __name__ == '__main__':
    rclpy.init(args=None)

    commander = Commander()
    cmd_vel_subscriber = CmdVel_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(cmd_vel_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

