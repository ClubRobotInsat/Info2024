#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from arm_interface.action import ExecArmAction

### CONSTANT ###
# ID for action
MOVE_HOME_POS = 1
MOVE_READY_POS = 2
MOVE_PUT_TO_STOCK = 3
MOVE_GET_FROM_STOCK = 4
MOVE_GRAB = 5
MOVE_RELEASE = 6

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
        # Initialize the status of the arm
        self.is_busy_ = False
        self.get_logger().info('Arm Controller Node Started')
    
    def arm_execute_callback(self, goal_handle : ServerGoalHandle):
        # Get the request for action
        action_id = goal_handle.request.action_id
        self.get_logger().info('Received request for action: %d' % action_id)
        # Execute the action
        ## TODO: Send command to Herkulex motor via can TX
        # Update goal final status
        goal_handle.succeed()
        # Send the result
        result = ExecArmAction.Result()
        result.action_result = True
        return result
        
    

def main(args=None):
    try:
        rclpy.init(args=args)
        arm_controller = ArmController()
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()