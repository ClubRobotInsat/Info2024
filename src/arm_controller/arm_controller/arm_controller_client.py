#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from arm_interface.action import ExecArmAction

### CONSTANT ###
# ID for action
MOVE_HOME_POS = 1
MOVE_READY_POS = 2
MOVE_PUT_TO_STOCK = 3
MOVE_GET_FROM_STOCK = 4
MOVE_GRAB = 5
MOVE_RELEASE = 6

class ArmControllerClient(Node):
    def __init__(self):
        super().__init__('arm_controller_client')
        self.exec_arm_action_client_ = ActionClient(
            self,
            ExecArmAction,
            "arm_action")
        self.get_logger().info('Arm Controller Client Started')
    
    def send_request(self, action_id):
        self.exec_arm_action_client_.wait_for_server()

        goal_msg = ExecArmAction.Goal()
        goal_msg.action_id = action_id
        self.get_logger().info('Sending request for action: %d' % action_id)
        self.exec_arm_action_client_.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self.goal_handle_ : ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info('Goal accepted')
            self.goal_handle_.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result received: %d' % result.action_result)

def main(args=None):
    try:
        action_request = 1
        rclpy.init(args=args)
        arm_controller_client = ArmControllerClient()
        arm_controller_client.send_request(action_request)
        rclpy.spin(arm_controller_client)
    except KeyboardInterrupt:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()