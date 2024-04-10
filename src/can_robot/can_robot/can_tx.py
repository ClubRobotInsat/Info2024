#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
import can

from can_raw_interfaces.msg import CanRaw,ServoCmd,MotorCmd,SensorCmd

commands={"servo_cmd":1, "motor_cmd":2,"sensor_cmd": 3}

class CanTx(Node):
    
    def __init__(self):
        """ CanTx constructor """
        super().__init__('can_tx')
        
        #Set up topics to subscribe to
        self.servo_cmd_subscriber = self.create_subscription(ServoCmd, 'servo_cmd',self.on_servo_cmd, 10)
        self.motor_cmd_subscriber = self.create_subscription(MotorCmd,'motor_cmd',self.on_motor_cmd,10)
        self.sensor_cmd_subscriber = self.create_subscription(SensorCmd,'sensor_cmd',self.on_sensor_cmd,10)
        
        #Set up topic to publish to
        self.can_raw_tx_publisher = self.create_publisher(CanRaw, 'can_raw_tx', 10)
        
    def on_sensor_cmd(self,msg):
        """ Upon receiving a command, this Callback is called """
        self.get_logger().info('I heard: "%s"' % msg.data)
        #Check data in the command and identify which type of command it is
        
        #Switch case on the type of command 
        
        #Construct the CanRaw Message 
        
        #Publish the CanRaw Message
    
    def on_motor_cmd(self,msg):
        """ Upon receiving a motor command, this Callback is called """
        self.get_logger().info('Motor command incoming')
        
    def on_servo_cmd(self,msg):
        """ Upon receiving a servo command, this Callback is called """
        self.get_logger().info('I heard: "%s"' % msg.angle)
        
    

def main(args=None):
    rclpy.init(args=args) #Initialise ROS2 communications

    can_tx_node = CanTx()

    rclpy.spin(can_tx_node)

    rclpy.shutdown() #Shutdown ROS2 communications


if __name__ == '__main__':
    main()
        
       
        
        