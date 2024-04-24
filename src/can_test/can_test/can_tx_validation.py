#!/usr/bin/env python3

import rclpy
import yaml
from rclpy.node import Node, Timer
import can

from can_interface.msg import ServoCmd,MotorCmd,SensorCmd

class CanTxValidation(Node):

    def __init__(self):
        """ CanTxValidation constructor """
        super().__init__('can_tx_validation') #Call to Node constructor with a parameter

        self.can_tx_validation_publisher = self.create_publisher(MotorCmd, 'motor_cmd', 10)
        
        #Get test file name from the command line, in an absolute form /home/ws/src/<file name> for example
        self.declare_parameter('file_name', rclpy.Parameter.Type.STRING)
        string = str(self.get_parameter('file_name').value)
        with open(string) as stream:
            try:
                test_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        for elt in test_data['motor_cmd']:
            motorCmdData = MotorCmd()
            print(elt[1])
            motorCmdData.dest = elt[0]
            motorCmdData.command_id = elt[1]
            motorCmdData.motor_id = elt[2]
            motorCmdData.direction = elt[3]
            motorCmdData.speed = elt[4]
            motorCmdData.extra = elt[5]
            self.can_tx_validation_publisher.publish(motorCmdData)
        
        for elt in test_data['servo_cmd']:
            #[TODO]
            pass
        
        for elt in test_data['sensor_cmd']:
            #[TODO]
            pass


def main(args=None):
    rclpy.init(args=args) #Initialise ROS2 communications

    can_tx_validation_node = CanTxValidation()

    rclpy.spin(can_tx_validation_node)

    rclpy.shutdown() #Shutdown ROS2 communications


if __name__ == '__main__':
    main()