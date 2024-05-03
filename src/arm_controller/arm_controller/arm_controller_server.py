#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from arm_interface.action import ExecArmAction
from can_interface.msg import ServoCmd
from enum import Enum

### CONSTANT ###
# ID for action
class ArmAction(Enum):
    MOVE_HOME_POS = 1
    MOVE_READY_POS = 2
    MOVE_PUT_TO_STOCK = 3
    MOVE_GET_FROM_STOCK = 4
    MOVE_GRAB = 5
    MOVE_RELEASE = 6

# ID arm STM32
ARM_STM_ID = 4

# ID for command
class ArmCmdID(Enum):
    CMD_ID_MOVE_HOME_POS = 19
    CMD_ID_MOVE_READY_POS = 20
    CMD_ID_MOVE_PUT_TO_STOCK = 21
    CMD_ID_MOVE_GET_FROM_STOCK = 22
    CMD_ID_MOVE_GRAB = 17
    CMD_ID_MOVE_RELEASE = 18

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.exec_arm_action_server_ = ActionServer(
            self,
            ExecArmAction,
            "ArmController",
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
        result = ExecArmAction.Result()
        result.action_result = True
        return result
    
    def send_arm_cmd(self, action_id):
        msg = ServoCmd()
        msg.dest = ARM_STM_ID
        #Switch case on the type of command 
        try:
            match action_id :
                case ArmAction.MOVE_HOME_POS.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_HOME_POS
                case ArmAction.MOVE_READY_POS.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_READY_POS
                case ArmAction.MOVE_PUT_TO_STOCK.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_PUT_TO_STOCK
                case ArmAction.MOVE_GET_FROM_STOCK.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_GET_FROM_STOCK
                case ArmAction.MOVE_GRAB.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_GRAB
                case ArmAction.MOVE_RELEASE.value:
                    msg.command_id = ArmCmdID.CMD_ID_MOVE_RELEASE
                case _:
                    pass
        except:
            self.get_logger().info('Invalid Herkulek action ID')
            return
        # Publish the message
        self.arm_pub_.publish(msg)
        

def main(args=None):
    try:
        rclpy.init(args=args)
        arm_controller = ArmController()
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()