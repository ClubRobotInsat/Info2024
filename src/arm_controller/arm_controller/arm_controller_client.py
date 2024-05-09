#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from arm_interface.action import ArmAction

# ### ArmAction ID ###
#                1 : Grab
#                2 : Release
#                3 : Move to home position
#                4 : Move to ready position
#                5 : Move to find objet low position
#                6 : Move to find object high position
#                7 : Put object to stock
#                8 : Get object from stock
#                9 : Place object on table

class ArmControllerClient(Node):
    def __init__(self):
        super().__init__('arm_controller_client')
        self.arm_action_client_ = ActionClient(
            self,
            ArmAction,
            "ArmAction")
        self.get_logger().info('Arm Controller Client Started')
        self.finish_arm_action = False
    
    def arm_send_request(self, action_id):
        self.arm_action_client_.wait_for_server()

        goal_msg = ArmAction.Goal()
        goal_msg.action_id = action_id
        self.get_logger().info('Sending request for action: %d' % action_id)
        self.arm_action_client_.send_goal_async(goal_msg).add_done_callback(self.arm_goal_response_callback)
    
    def arm_goal_response_callback(self, future):
        self.goal_handle_ : ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info('Goal accepted')
            self.goal_handle_.get_result_async().add_done_callback(self.arm_get_result_callback)
        else:
            self.get_logger().info('Goal rejected')

    def arm_get_result_callback(self, future):
        result = future.result().result
        if result.action_result == True:
            self.get_logger().info('Action succeeded')
            self.finish_arm_action = True
        else:
            self.get_logger().info('Action failed')
            self.finish_arm_action = True
        self.get_logger().info('Result received: %d' % result.action_result)

def main(args=None):
    try:
        action_request = 1
        rclpy.init(args=args)
        arm_controller_client = ArmControllerClient()
        for i in range(10):
            arm_controller_client.arm_send_request(action_request+i)
        rclpy.spin(arm_controller_client)
    except KeyboardInterrupt:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()