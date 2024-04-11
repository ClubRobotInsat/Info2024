#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from enum import Enum
import can

from can_raw_interfaces.msg import CanRaw,ServoCmd,MotorCmd,SensorCmd

commands={"servo_cmd":1, "motor_cmd":2,"sensor_cmd": 3}
tab_ids={"raspi":1, "herkulex":2, "base roulante":3}
reversed_tab_ids=dict((v,k) for (k,v) in tab_ids.items())

class MotorCommands(Enum): 
    STOP = 0
    PING = 1
    SET_SPEED = 2
    GET_ACK = 3
    SET_DIR = 4
    GET_SPEED = 5
    GET_DIR = 6

class ServoCommands(Enum):
    STOP = 0
class SensorCommands(Enum):
    STOP = 0


def generate_header(prio,dest,origine):
    header = 0
    header = (header ^ (prio << 8))
    header = (header ^ (dest << 4))
    header = (header ^ (origine))
    return header



class CanTx(Node):
    
    def __init__(self):
        """ CanTx constructor """
        super().__init__('can_tx')
        
        #Set up topics to subscribe to
        self.servo_cmd_subscriber = self.create_subscription(ServoCmd, 'servo_cmd',self.on_servo_cmd, 10)
        self.motor_cmd_subscriber = self.create_subscription(MotorCmd,'motor_cmd',self.on_motor_cmd,10)
        self.sensor_cmd_subscriber = self.create_subscription(SensorCmd,'sensor_cmd',self.on_sensor_cmd,10)
        
        #Set up topic to publish to [Potentially add an error flag topic for when this node detects an error]
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
        # Publish the message to the appropriate topic
        canRawMsg = CanRaw()
        prio, dest, origine = 0,1,1
        header = generate_header(prio,dest,origine)
        canRawMsg.arbitration_id = header
        # [Code factorisation possible] [Error Handling necessary]
        match msg.command_id : 
            case MotorCommands.STOP.value:
                canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                
            case MotorCommands.PING.value:
                canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                
            case MotorCommands.SET_SPEED.value:
                # Convert speed into adequate data type [Must be improved for float compatibility]
                speed = msg.speed #(m/s) 
                canRawMsg.data = [msg.command_id,msg.motor_id,((speed >> 24) & 255),((speed >> 16) & 255 ),((speed >> 8) & 255),(speed & 255),0,0]
                
            case MotorCommands.GET_ACK.value:
                speed = msg.speed #(m/s) 
                canRawMsg.data = [msg.command_id,msg.motor_id,((speed >> 24) & 255),((speed >> 16) & 255 ),((speed >> 8) & 255),(speed & 255),0,0]
    
            case MotorCommands.SET_DIR.value:
                canRawMsg.data = [msg.command_id,msg.motor_id,msg.direction,0,0,0,0,0]
    
            case MotorCommands.GET_SPEED.value:
                canRawMsg.data = [msg.command_id,msg.motor_id,0,0,0,0,0,0]
                
            case MotorCommands.GET_DIR.value:
                canRawMsg.data = [msg.command_id,msg.motor_id,0,0,0,0,0,0]
                
            case _:
                pass
            
        canRawMsg.err_flag = 0
        canRawMsg.rtr_flag = 0
        canRawMsg.eff_flag = 0

        self.can_raw_tx_publisher.publish(canRawMsg)
        
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
        
       
        
        