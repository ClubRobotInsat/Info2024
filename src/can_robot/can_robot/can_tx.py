#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from enum import Enum
import can
import numpy as np

from can_raw_interfaces.msg import CanRaw,ServoCmd,MotorCmd,SensorCmd

commands={"servo_cmd":1, "motor_cmd":2,"sensor_cmd": 3}
tab_ids={"raspi":1, "herkulex":2, "base roulante":3}
reversed_tab_ids=dict((v,k) for (k,v) in tab_ids.items())

class MotorCommands(Enum): 
    STOP = 0
    PING = 1
    SET_SPEED = 2
    GET_SPEED_ACK = 3
    SET_DIR = 4
    GET_SPEED = 5
    GET_DIR = 6

class ServoCommands(Enum):
    STOP = 0
    PING = 1
    SET_ANGLE = 2
    GET_ANGLE = 3
    GET_ANGLE_ACK = 4
    SET_SPEED = 5
    GET_SPEED = 6
    GET_SPEED_ACK = 7
    SET_SPIN_DURATION = 8
    CHANGE_MODE = 9
    GET_MODE = 10
    SET_TORQUE = 11
    GET_TORQUE = 12
    REBOOT = 13
    CLEAR_ERROR = 14
    GET_ERROR = 15
    GET_STATUS = 16
    GRAB = 17
    RELEASE = 18
class SensorCommands(Enum):
    STOP = 0
    PING = 0


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
        
        #Set the float multiplier
        self.float_multiplier = 10
        
    def on_sensor_cmd(self,msg):
        """ Upon receiving a command, this Callback is called """
        self.get_logger().info('I heard: "%s"' % msg.data)
        canRawMsg = CanRaw()
        origin = 0 #[0 for raspi?]
        header = generate_header(msg.prio,msg.dest,origin)
        canRawMsg.arbitration_id = header
        
        #Switch case on the type of command 
        
        try:
            match msg.command_id :
                
                case SensorCommands.STOP.value: 
                    canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                case SensorCommands.PING.value: 
                    canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                case _ :
                    #[Add an error flag for unrecognised command?]
                    pass
        except Exception as err :
            print(f"Unexpected {err=}, {type(err)=}")
            pass
            
        #Construct the CanRaw Message 
        canRawMsg.err_flag = 0
        canRawMsg.rtr_flag = 0
        canRawMsg.eff_flag = 0
        
        #Publish the CanRaw Message
        self.can_raw_tx_publisher.publish(canRawMsg)
    
    def on_motor_cmd(self,msg):
        """ Upon receiving a motor command, this Callback is called """
        self.get_logger().info('Motor command incoming')
        # Publish the message to the appropriate topic
        canRawMsg = CanRaw()
        origin = 0 #[0 for raspi?]
        header = generate_header(msg.prio,msg.dest,origin)
        canRawMsg.arbitration_id = header
        # [Code factorisation possible] [Error Handling necessary]
        try: 
            match msg.command_id : 
                case MotorCommands.STOP.value:
                    canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                    
                case MotorCommands.PING.value:
                    canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                    
                case MotorCommands.SET_SPEED.value:
                    # Convert speed into adequate data type [Must be improved for float compatibility]
                    speed = int(np.ceil(msg.speed*self.float_multiplier)) #(m/s) 
                    canRawMsg.data = [msg.command_id,msg.motor_id,((speed >> 24) & 255),((speed >> 16) & 255 ),((speed >> 8) & 255),(speed & 255),0,0]
                    
                case MotorCommands.GET_SPEED_ACK.value:
                    speed = int(np.ceil(msg.speed*self.float_multiplier)) #(m/s) 
                    canRawMsg.data = [msg.command_id,msg.motor_id,((speed >> 24) & 255),((speed >> 16) & 255 ),((speed >> 8) & 255),(speed & 255),0,0]
        
                case MotorCommands.SET_DIR.value:
                    canRawMsg.data = [msg.command_id,msg.motor_id,msg.direction,0,0,0,0,0]
        
                case MotorCommands.GET_SPEED.value:
                    canRawMsg.data = [msg.command_id,msg.motor_id,0,0,0,0,0,0]
                    
                case MotorCommands.GET_DIR.value:
                    canRawMsg.data = [msg.command_id,msg.motor_id,0,0,0,0,0,0]
                    
                case _:
                    pass
        except Exception as err:
            print(f"Unexpected {err=}, {type(err)=}")
            pass
            
        canRawMsg.err_flag = 0
        canRawMsg.rtr_flag = 0
        canRawMsg.eff_flag = 0

        self.can_raw_tx_publisher.publish(canRawMsg)
        
    def on_servo_cmd(self,msg):
        """ Upon receiving a servo command, this Callback is called """
        self.get_logger().info('I heard: "%s"' % msg.angle)
        canRawMsg = CanRaw()
        origin = 0 #[0 for raspi?]
        header = generate_header(msg.prio,msg.dest,origin)
        canRawMsg.arbitration_id = header
        try: 
            match msg.command_id:
                case ServoCommands.STOP.value:
                    canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                    
                case ServoCommands.PING.value:
                    canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                    
                case ServoCommands.SET_ANGLE.value:
                    angle = int(np.ceil(msg.angle*self.float_multiplier))
                    canRawMsg.data = [msg.command_id,msg.servo_id,((angle >> 24) & 255),((angle >> 16) & 255 ),((angle >> 8) & 255),(angle & 255),0,0,0,0,0]
                    
                case ServoCommands.GET_ANGLE.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,0,0,0,0,0,0]
                    
                case ServoCommands.GET_ANGLE_ACK.value:
                    angle = int(np.ceil(msg.angle*self.float_multiplier))
                    canRawMsg.data = [msg.command_id,msg.servo_id,((angle >> 24) & 255),((angle >> 16) & 255 ),((angle >> 8) & 255),(angle & 255),0,0]
                
                case ServoCommands.SET_SPEED.value:
                    speed = int(np.ceil(msg.speed*self.float_multiplier))
                    canRawMsg.data = [msg.command_id,msg.servo_id,((speed >> 24) & 255),((speed >> 16) & 255 ),((speed >> 8) & 255),(speed & 255),0,0]
                
                case ServoCommands.GET_SPEED.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,0,0,0,0,0,0]
                
                case ServoCommands.GET_SPEED_ACK.value:
                    speed = int(np.ceil(msg.speed*self.float_multiplier))
                    canRawMsg.data = [msg.command_id,msg.servo_id,((speed >> 24) & 255),((speed >> 16) & 255 ),((speed >> 8) & 255),(speed & 255),0,0]
                
                case ServoCommands.SET_SPIN_DURATION.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,((msg.duration >> 8) & 255),(msg.duration & 255),0,0,0,0]
                
                case ServoCommands.CHANGE_MODE.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,msg.mode,0,0,0,0,0]
                
                case ServoCommands.GET_MODE.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,msg.mode,0,0,0,0,0]
                
                case ServoCommands.SET_TORQUE.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,msg.torque,0,0,0,0,0]
                
                case ServoCommands.GET_TORQUE.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,0,0,0,0,0,0]
                
                case ServoCommands.REBOOT.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,0,0,0,0,0,0]
                
                case ServoCommands.CLEAR_ERROR.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,0,0,0,0,0,0]
                
                case ServoCommands.GET_ERROR.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,0,0,0,0,0,0]
                
                case ServoCommands.GET_STATUS.value:
                    canRawMsg.data = [msg.command_id,msg.servo_id,0,0,0,0,0,0]
                
                case ServoCommands.GRAB.value:
                    canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                
                case ServoCommands.RELEASE.value:
                    canRawMsg.data = [msg.command_id,0,0,0,0,0,0,0]
                
                case _ :
                    pass
        
        except Exception as err:
            print(f"Unexpected {err=}, {type(err)=}")
            pass
        
        canRawMsg.err_flag = 0
        canRawMsg.rtr_flag = 0
        canRawMsg.eff_flag = 0

        self.can_raw_tx_publisher.publish(canRawMsg)
            
    
        
    

def main(args=None):
    rclpy.init(args=args) #Initialise ROS2 communications

    can_tx_node = CanTx()

    rclpy.spin(can_tx_node)

    rclpy.shutdown() #Shutdown ROS2 communications


if __name__ == '__main__':
    main()
        
       
        
        