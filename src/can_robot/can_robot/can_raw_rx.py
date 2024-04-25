#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
import can

from can_interface.msg import CanRaw

class CanRawRx(Node):
    def __init__(self):

        super().__init__('can_raw_rx')

        # Subscribe to all data on the can_robot bus
        # with can.interface.Bus(interface='socketcan', channel='can0', receive_own_messages=True) as bus:
        #     notifier = can.Notifier(bus, [self.on_message])

        self.bus = can.interface.Bus(interface='socketcan',
                                     channel='can0',
                                     receive_own_messages=True)

        notifier = can.Notifier(self.bus, [self.on_message])
        # Publish the message to the appropriate topic
        # CanRawRx is the message type with the following fields:
        #  - arbitration_id: int
        #  - data: bytes [8]
        #  - uint8 err_flag
        #  - uint8 rtr_flag
        #  - uint8 eff_flag
        self.can_raw_rx_publisher = self.create_publisher(CanRaw, 'can_raw_rx', 10)

    def on_message(self, msg):
        """
        Callback function for the can_robot bus
        """
        # Publish the message to the appropriate topic
        canRawMsg = CanRaw()
        canRawMsg.arbitration_id = msg.arbitration_id
        canRawMsg.data = msg.data
        canRawMsg.err_flag = msg.is_error_frame
        canRawMsg.rtr_flag = msg.is_remote_frame
        canRawMsg.eff_flag = msg.is_extended_id

        self.can_raw_rx_publisher.publish(canRawMsg)
        self.get_logger().info('Received message: {0}'.format(msg))


def main(args=None):
    rclpy.init(args=args)

    can_raw_rx_node = CanRawRx()

    rclpy.spin(can_raw_rx_node)

    r = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            r.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
