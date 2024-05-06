#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from chain_interface.action import ControlChain
from can_interface.msg import ServoCmd
from enum import Enum

### CONSTANT ###
# ID for action
class ChainActionID(Enum):
    MOVE_TO_NEXT_RING = 1
    MOVE_TO_PREVIOUS_RING = 2

# ID chain STM32
CHAIN_STM_ID = 4

# ID for command
class ChainCmdID(Enum):
    CMD_ID_MOVE_TO_NEXT_RING = 26
    CMD_ID_MOVE_TO_PREVIOUS_RING = 27  


class ChainControllerServerNode(Node):
    def __init__(self):
        super().__init__('chain_controller_server')
        self.chain_controller_server = ActionServer(
            self,
            ControlChain,
            'control_chain',
            execute_callback=self.execute_callback)
        # Create a publisher to send arm command to CAN bus
        self.chain_pub_ = self.create_publisher(ServoCmd, 'servo_cmd', 10)
        # Initialize the status of the arm
        self.is_busy_ = False
        self.get_logger().info("Chain Controller Server has been started")

    
    def execute_callback(self, goal_handle:ServerGoalHandle):
        #Get request from the goal
        action_id = goal_handle.request.chain_action_id
        self.get_logger().info('Received request for action: %d' % action_id)
        self.send_chain_cmd(action_id)
        #Execute the action
        done = False
        #Set the result
        goal_handle.succeed()
        #Send the result
        result = ControlChain.Result()
        result.success = done
        return result
    
    def send_chain_cmd(self, id_action:int):
        cmd_msg = ServoCmd()
        cmd_msg.dest = CHAIN_STM_ID
        try:
            match id_action :
                case ChainActionID.MOVE_TO_NEXT_RING.value:
                    cmd_msg.command_id = ChainCmdID.CMD_ID_MOVE_TO_NEXT_RING.value
                    self.get_logger().info('Sending command to move to next ring')
                case ChainActionID.MOVE_TO_PREVIOUS_RING.value:
                    cmd_msg.command_id = ChainCmdID.CMD_ID_MOVE_TO_PREVIOUS_RING.value
                    self.get_logger().info('Sending command to move to previous ring')
                case _:
                    self.get_logger().info('Invalid action ID')
        except Exception as err :
            print(f"Unexpected {err=}, {type(err)=}")
            pass
        # Publish the message
        if(cmd_msg.command_id != 0):
            self.get_logger().info('Sending command: %d' % cmd_msg.command_id)
            self.chain_pub_.publish(cmd_msg)            
        else:
            self.get_logger().info('Invalid action ID: %d' % id_action)
    

def main(args=None):
    rclpy.init(args=args)

    chain_controller_server = ChainControllerServerNode()

    rclpy.spin(chain_controller_server)

    chain_controller_server.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main