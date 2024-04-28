#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray

import threading
import numpy as np

from can_interface.msg import MotorCmd

# Create dictionary to store id of each motor
motor_id = {
    'left': 0,
    'front': 1,
    'right': 2,
    'back': 3
}

prev_data = np.array([0.0,0.0,0.0,0.0])

class FVCSubscriberCan(Node):
    def __init__(self):
        super().__init__('fvc_subscriber_can')
        self.subscriber = self.create_subscription(Float64MultiArray, '/forward_velocity_controller/commands', self.listener_callback, 10)
        self.publisher = self.create_publisher(MotorCmd, '/motor_cmd', 10)
    def listener_callback(self, data):
        global prev_data
        # self.get_logger().info('I heard: "%s"' % data.data)

        # For each motor, we have to make a motorMsg and publish it
        # LIAAAAAAAAM
        if not np.array_equal(data.data, prev_data):
            for i in range(4):
                # Get the motor id from the dictionary
                motor = list(motor_id.keys())[list(motor_id.values()).index(i)]

                motorMsg = MotorCmd()
                if motor == 'left' or motor == 'right':
                    motorMsg.dest = 2
                    if motor == 'left':
                        motorMsg.motor_id = 0
                    if motor == 'right':
                        motorMsg.motor_id = 1
                else:
                    motorMsg.dest = 3
                    if motor == 'front':
                        motorMsg.motor_id = 0
                    if motor == 'back':
                        motorMsg.motor_id = 1
                motorMsg.command_id = 3 # Set speed

                if data.data[i] > 0:
                    motorMsg.direction = 1
                else:
                    motorMsg.direction = 0
                # 0.9 m/s is the max speed of the robot
                motorMsg.speed = data.data[i] * 1000.0 # Convert from m/s to mm/s
                motorMsg.extra = 0
                self.publisher.publish(motorMsg)
            prev_data = data.data


if __name__ == '__main__':
    rclpy.init(args=None)

    fvc_subscribercan = FVCSubscriberCan()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(fvc_subscribercan)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = fvc_subscribercan.create_rate(1000)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

