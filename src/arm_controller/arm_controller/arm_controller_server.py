#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from arm_interface.action import ArmAction
from can_interface.msg import ServoCmd
from enum import Enum

### CONSTANT ###
# ID for action
class ArmActionID(Enum):
    MOVE_GRAB = 1
    MOVE_RELEASE = 2
    MOVE_HOME_POS = 3
    MOVE_READY_POS = 4
    MOVE_FIND_OBJECT_LOW = 5
    MOVE_FIND_OBJECT_HIGH = 6
    MOVE_PUT_TO_STOCK = 7
    MOVE_GET_FROM_STOCK = 8
    MOVE_PLACE_OBJECT = 9

# ID arm STM32
ARM_STM_ID = 4

# ID for command
class ArmCmdID(Enum):
    CMD_ID_MOVE_GRAB = 17
    CMD_ID_MOVE_RELEASE = 18
    CMD_ID_MOVE_HOME_POS = 19
    CMD_ID_MOVE_READY_POS = 20
    CMD_ID_MOVE_FIND_OBJECT_LOW = 21
    CMD_ID_MOVE_FIND_OBJECT_HIGH = 22
    CMD_ID_MOVE_PUT_TO_STOCK = 23
    CMD_ID_MOVE_GET_FROM_STOCK = 24
    CMD_ID_MOVE_PLACE_OBJECT = 25   

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.arm_action_server_ = ActionServer(
            self,
            ArmAction,
            "ArmAction",
            execute_callback=self.arm_execute_callback)
        # Initialize current position of arm
        self.current_pos_ = [0, 0, 0]
        # Create a publisher to send arm command to CAN bus
        self.arm_pub_ = self.create_publisher(ServoCmd, 'servo_cmd', 10)
        # Initialize the status of the arm
        self.is_busy_ = False
        self.get_logger().info('Arm Controller Node Started')
        
    def arm_execute_callback(self, goal_handle : ServerGoalHandle):
        # Get the request for action
        action_id = goal_handle.request.action_id
        self.get_logger().info('Received request for action: %d' % action_id)
        # Execute the action
        self.send_arm_cmd(action_id)
        # Update goal final status
        goal_handle.succeed()
        # Send the result
        result = ArmAction.Result()
        result.action_result = True
        return result
    
    def send_arm_cmd(self, id_action):
        msg = ServoCmd()
        msg.dest = ARM_STM_ID
        #Switch case on the type of command 
        #print(f"Action ID: {id_action}")
        try:
            match id_action :
                case ArmActionID.MOVE_HOME_POS.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_HOME_POS.value
                case ArmActionID.MOVE_READY_POS.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_READY_POS.value
                case ArmActionID.MOVE_FIND_OBJECT_LOW.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_FIND_OBJECT_LOW.value
                case ArmActionID.MOVE_FIND_OBJECT_HIGH.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_FIND_OBJECT_HIGH.value
                case ArmActionID.MOVE_PUT_TO_STOCK.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_PUT_TO_STOCK.value
                case ArmActionID.MOVE_GET_FROM_STOCK.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_GET_FROM_STOCK.value
                case ArmActionID.MOVE_PLACE_OBJECT.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_PLACE_OBJECT.value
                case ArmActionID.MOVE_GRAB.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_GRAB.value
                case ArmActionID.MOVE_RELEASE.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_RELEASE.value
                case _:
                    pass
        except Exception as err :
            print(f"Unexpected {err=}, {type(err)=}")
            pass
        # Publish the message
        if(msg.command_id != 0):
            self.get_logger().info('Sending command: %d' % msg.command_id)
            self.arm_pub_.publish(msg)            
        else:
            self.get_logger().info('Invalid action ID: %d' % id_action)
        

def main(args=None):
    try:
        rclpy.init(args=args)
        arm_controller = ArmController()
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()