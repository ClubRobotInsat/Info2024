import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile

from interfaces.action import Behaviour


class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')

        # Create the action server
        self._action_server = ActionServer(
            self,
            Behaviour,
            'my_action',  # Action name
            self.handle_goal,
            cancel_callback=self.handle_cancel,
        )

    def handle_goal(self, goal_request):

        self.get_logger().info(goal_request)
        self.get_logger().info('Received goal request: ' + str(goal_request.goal))

        # Create a feedback message
        feedback_msg = Behaviour.Feedback()
        feedback_msg.feedback = 0

        # Create a result message
        result_msg = Behaviour.Result()
        result_msg.result = 0

        # Perform the action (replace this with your custom logic)
        for i in range(1, goal_request.goal + 1):
            feedback_msg.feedback = i
            self._action_server.update_feedback(feedback_msg)

        result_msg.result = goal_request.goal
        self._action_server.succeed(result_msg)

        self.get_logger().info('Action completed successfully')

    def handle_cancel(self, goal_handle):
        self.get_logger().info('Goal canceled')
        return ActionServer.CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    action_server = MyActionServer()

    rclpy.spin(action_server)

    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
