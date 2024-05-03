#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from chain_interface.action import ControlChain

class ChainControllerServerNode(Node):
    def __init__(self):
        super().__init__('chain_controller_server')
        self.chain_controller_server = ActionServer(
            self,
            ControlChain,
            'control_chain',
            execute_callback=self.execute_callback)
        self.get_logger().info("Chain Controller Server has been started")

    
    def execute_callback(self, goal_handle:ServerGoalHandle):
        #Get request from the goal
        chain_action_num = goal_handle.request.chain_action_id
        
        #Execute the action
        self.get_logger().info(f"Executing the action {chain_action_num}")
        done = False
        if chain_action_num == 1:
            for i in range(300000000):
                pass
            self.get_logger().info("Action 1 has been executed")
            done = True
        elif chain_action_num == 2:
            self.get_logger().info("Action 2 has been executed")
            done = True
        elif chain_action_num == 0:
            self.get_logger().info("Action 0 has been executed")
            done = True
        elif chain_action_num == -1:
            self.get_logger().info("Action -1 has been executed")
            done = True
        else:
            self.get_logger().info("Invalid action number")
            done = False
        
        #Set the result
        goal_handle.succeed()
        
        #Send the result
        result = ControlChain.Result()
        result.success = done
        return result
    

def main(args=None):
    rclpy.init(args=args)

    chain_controller_server = ChainControllerServerNode()

    rclpy.spin(chain_controller_server)

    chain_controller_server.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main