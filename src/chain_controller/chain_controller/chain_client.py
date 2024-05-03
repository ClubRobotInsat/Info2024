#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from chain_interface.action import ControlChain

class ChainClientNode(Node):
    def __init__(self):
        super().__init__('chain_client')
        self.chain_client = ActionClient(
            self,
            ControlChain,
            'control_chain'
        )
        
    def send_goal(self, chain_action_num):
        
        self.chain_client.wait_for_server()
        
        goal = ControlChain.Goal()
        goal.chain_action_id = chain_action_num
        
        self.get_logger().info(f"Sending goal {chain_action_num}")
        self.chain_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self.goal_handle_:ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal accepted")
            self.goal_handle_.get_result().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal rejected")
            
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.success}")
        
        
def main(args=None):
    rclpy.init(args=args)
    chain_client = ChainClientNode()
    chain_client.send_goal(1)
    chain_client.send_goal(7)
    chain_client.send_goal(-1)
    rclpy.spin(chain_client)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main