import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile

from interfaces.action import Behaviour


class MyActionClient(Node):
    def __init__(self):
        super().__init__('my_action_client')

        # Create the action client
        self._action_client = ActionClient(
            self,
            Behaviour,
            'my_action',  # Action name
        )

        # Wait for the action server to become available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for the action server...')

    def send_goal(self, goal):
        goal_msg = Behaviour.Goal()
        goal_msg.goal = goal

        # Send the goal request to the action server
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Wait for the result
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: ' + str(feedback_msg.feedback))

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the server')
            return

        self.get_logger().info('Goal accepted by the server')

        # Wait for the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Action completed with result: ' + str(result))


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    # Send a goal request (replace 10 with your desired goal value)
    action_client.send_goal("10")

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
