#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
import can

from can_raw_interfaces.msg import CanRaw

class CanRawTx(Node):

    def __init__(self):
        """ CanRawTx constructor """
        super().__init__('can_raw_tx') #Call to Node constructor with a parameter

        # Sets up a link to a linux socketcan CAN bus interface
        self.can_bus = can.ThreadSafeBus(interface='socketcan',
                                     channel='can0',
                                     bitrate=125000,
                                     receive_own_messages=False)

        # Sets up a bus for listening to the CAN bus
        self.can_bus_notifier = can.ThreadSafeBus(interface='socketcan',
                                     channel='can0',
                                     bitrate=125000,
                                     receive_own_messages=False)

        self.notifier = can.Notifier(self.can_bus_notifier, [self.on_message])

        self.can_raw_tx_subscriber = self.create_subscription(CanRaw, 'can_raw_tx',self.on_topic_data, 10) #Subscription to can_raw_tx with CanRaw interface definition

    def on_topic_data(self,can_raw_tx_msg):
        """ Callback which is called whenever raw transmission data arrives in the can_raw_tx topic """

        self.send_single_msg(can_raw_tx_msg.arbitration_id,can_raw_tx_msg.data,can_raw_tx_msg.err_flag,can_raw_tx_msg.rtr_flag,can_raw_tx_msg.eff_flag)

    def on_message(self, msg):
        """
        Callback which is called whenever the CAN interface detects incoming traffic on the BUS
        """
        self.get_logger().info('Received message: {0}'.format(msg))

    def send_single_msg(self,arbitration_id,data,err_flag,rtr_flag,eff_flag):
        """ Sends a single message on the CAN bus """

        msg = can.Message(
            arbitration_id=arbitration_id, data=data, is_extended_id=False
        )
        try:
            self.can_bus.send(msg)
            print(f"Message sent on {self.can_bus.channel_info}")

        except can.CanError:
            print("Message NOT sent")

def main(args=None):
    rclpy.init(args=args) #Initialise ROS2 communications

    can_raw_tx_node = CanRawTx()

    rclpy.spin(can_raw_tx_node)

    rclpy.shutdown() #Shutdown ROS2 communications


if __name__ == '__main__':
    main()