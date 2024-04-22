#!/usr/bin/env python3

import rclpy

from rclpy.node import Node, Timer
import can

from can_raw_interfaces.msg import ServoCmd,MotorCmd,SensorCmd

class CanTxValidation(Node):

    def __init__(self):
        """ CanTxValidation constructor """
        super().__init__('can_tx_validation') #Call to Node constructor with a parameter

        self.can_tx_validation_publisher = self.create_publisher(MotorCmd, 'motor_cmd', 10)
        
        #[Add timer callback] 
        
        for i in range(10):
            motorCmdData = MotorCmd()
            motorCmdData.dest = 1
            motorCmdData.command_id = 2
            motorCmdData.motor_id = 3
            motorCmdData.direction = i
            motorCmdData.speed = 5.0
            motorCmdData.extra = 6
            self.can_tx_validation_publisher.publish(motorCmdData)


def main(args=None):
    rclpy.init(args=args) #Initialise ROS2 communications

    can_tx_validation_node = CanTxValidation()

    rclpy.spin(can_tx_validation_node)

    rclpy.shutdown() #Shutdown ROS2 communications


if __name__ == '__main__':
    main()