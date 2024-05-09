#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from time import sleep
from std_msgs.msg import Bool
from can_interface.msg import ArmFeedback, Tirette, Enemy
from sensor_msgs.msg import JointState

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from arm_interface.action import ArmAction

class Homologation_Bras(Node):
    '''
    This class is the node for the homologation.

    What the robot should do:
    - Start moving forward
    - Stop moving forward
    - Move the arm and the chain
    - Go to the final place
    - Start on the left, go to the right

    - If enemy is detected, stop the robot
    '''

    def __init__(self):
        super().__init__('homologation')
        self.get_logger().info('Homologation node has been started')

        # Make a publisher for /cmd_vel 
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to /tirette to start the robot
        self.tirette_subscriber = self.create_subscription(Tirette, 'tirette', self.start_callback, 10)
        self.motor_subscriber = self.create_subscription(JointState, 'motors_feedback', self.motor_callback, 10)

        # Subscribe to /enemy to get info when an enemy is detected
        self.enemy_subscriber = self.create_subscription(Enemy, 'enemy', self.enemy_callback, 10)
        
        self.get_logger().info('Homologation node initialized')

        self.arm_action_client_ = ActionClient(
            self,
            ArmAction,
            "ArmAction")
        self.get_logger().info('Arm Controller Client Started')
        self.finish_arm_action = False

        self.allowed_moving = True
        self.move_timer = None

        self.sequence_index = 0

        # 1st parameter -> BR: 0        ARM (vs Intel x86): 1
        # 2st parametrr -> BR: vel_x    Arm: Action_ID (see ArmAction ID)
        # 3st parameter -> BR: vel_y    Arm: Not used
        self.instructions = [
            # (0.0, 0.15, 0.5),
            (0,0.0, 0.30, 1.5),  # GO FRONT     45cm
            (0,0.0, 0.0, 0.2),   # STOP
            (0,0.30, 0.0, 2.0),  # GO RIGHT     60cm
            # Move arm and chain
            (0,0.0, 0.0, 0.5),  # STOP

            # Mouvement bras
            (1, 4), # Move to ready position (after init, ready to do actions, e.g: solar panel)
            # (1, 5), # Move to Find object low position
            # (1, 1), # Grab
            # Go to final area
            (0,0.30, 0.0, 3.5), # GO RIGHT      105cm

            # (0.30, 0.0, 3.0),
            (0,0.0, 0.0, 0.2),  # STOP
            (0,0.0, -0.30, 1.5) # GO BACKWARD   45 cm
        ]

    def execute_next_instruction(self):
        if self.sequence_index < len(self.instructions):
            if self.move_timer is not None:
                self.move_timer.destroy()
            if self.allowed_moving:
                self.get_logger().info('Starting next instruction')
                match self.instruction[self.sequence_index][0]:
                    case 0: # Base roulante
                        instruction = self.instructions[self.sequence_index][1:] # 3 params (all)
                        self.move(*instruction)
                    case 1: # Arm
                        instruction = self.instructions[self.sequence_index][1] # 1 params
                        self.arm_move(*instruction)
                self.sequence_index += 1


        else:
            self.stop()
            self.get_logger().info('All instructions executed')
            self.move_timer.destroy()

    def start_callback(self, msg):
        self.get_logger().info('Start message received')
        if msg.go:
            self.get_logger().info('Start message is True')
            self.sequence_index = 0
            self.allowed_moving = True

            self.execute_next_instruction() # Recursive call -> call at the end of this scope
        else:
            self.get_logger().info('Start message is False -> Stop the robot')
            self.stop()

    def move_timer_callback(self):
        self.stop()
        if self.move_timer is not None:
            self.move_timer.destroy()

    def move(self, speed_x=0.0, speed_y=0.0, duration=1.0):
        # Move the robot
        twist = Twist()
        twist.linear.x = speed_x
        twist.linear.y = speed_y
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)
        self.get_logger().info(f"Robot should be moving at speed: x={speed_x} ; y={speed_y} for {duration} s.")

        self.move_timer = self.create_timer(duration, lambda: self.execute_next_instruction())


    def motor_callback(self,msg):
        pass

    def stop(self):
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)
        self.get_logger().info('Robot should be stopped')

    ####################### LIDAR Detection ###########################

    def enemy_callback(self, msg):
        self.allowed_moving = False
        self.stop()

    ######################## Arm Action client ########################
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
    def arm_move(self, arm_action_id = 1):
        self.arm_send_request(arm_action_id)
        # while self.finish_arm_action == False:
        #     pass
    
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
    rclpy.init(args=args)

    node = Homologation_Bras()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
