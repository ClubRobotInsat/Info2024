#!/usr/bin/env python3

import rclpy
import time
import yaml
from rclpy.node import Node, Timer
import can
import threading

from can_interface.msg import ServoCmd,MotorCmd,SensorCmd

class CanTxValidation(Node):

    def __init__(self):
        """ CanTxValidation constructor """
        super().__init__('can_tx_validation') #Call to Node constructor with a parameter

        self.motor_cmd_publisher = self.create_publisher(MotorCmd, 'motor_cmd', 10)
        self.servo_cmd_publisher = self.create_publisher(ServoCmd, 'servo_cmd', 10)
        self.sensor_cmd_publisher = self.create_publisher(SensorCmd, 'sensor_cmd', 10)
        
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
            motorCmdData.dest = elt[0]
            motorCmdData.command_id = elt[1]
            motorCmdData.motor_id = elt[2]
            motorCmdData.direction = elt[3]
            motorCmdData.speed = elt[4]
            motorCmdData.extra = elt[5]
            time.sleep(0.05)
            self.motor_cmd_publisher.publish(motorCmdData)
                
        for elt in test_data['servo_cmd']:
            servoCmdData = ServoCmd()
            servoCmdData.dest = elt[0]
            servoCmdData.command_id = elt[1]
            servoCmdData.servo_id = elt[2]
            servoCmdData.angle = elt[3]
            servoCmdData.speed = elt[4]
            servoCmdData.mode = elt[5]
            servoCmdData.torque = elt[6]
            servoCmdData.duration = elt[7]
            time.sleep(0.05)
            self.servo_cmd_publisher.publish(servoCmdData)
        
        for elt in test_data['sensor_cmd']:
            sensorCmdData = SensorCmd()
            sensorCmdData.dest = elt[0]
            sensorCmdData.command_id = elt[1]
            sensorCmdData.imu_id = elt[2]
            sensorCmdData.tof_id = elt[3]
            time.sleep(0.05)
            self.sensor_cmd_publisher.publish(sensorCmdData)
            
            

def main(args=None):
    rclpy.init(args=args) #Initialise ROS2 communications

    can_tx_validation_node = CanTxValidation()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(can_tx_validation_node, ), daemon=True)
    thread.start()
    
    try:
        while rclpy.ok():
            time.sleep(500)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown() #Shutdown ROS2 communications


if __name__ == '__main__':
    main()