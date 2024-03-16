#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
import can

from can_raw_interfaces.msg import CanRaw

class Test(Node):
    def __init__(self):

        super().__init__('can_raw_rx_test')

        # Subscribe to all data on the can_robot bus
        # with can.interface.Bus(interface='socketcan', channel='can0', receive_own_messages=True) as bus:
        #     notifier = can.Notifier(bus, [self.on_message])

        self.sub = self.create_subscription(CanRaw, 'can_raw_rx', self.on_message, 10)

    def on_message(self, msg):
        """
        Callback function for the can_robot bus
        """
        self.get_logger().info('Received message: {0}'.format(msg))



def main(args=None):
    rclpy.init(args=args)

    test = Test()

    rclpy.spin(test)

    r = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            r.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
