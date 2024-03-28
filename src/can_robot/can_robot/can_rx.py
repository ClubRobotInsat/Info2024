#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
import can

from sensor_msgs.msg import Imu, JointState, Range
from can_interface.msg import ArmFeedback, MotorsFeedback




##########################################################################################
tab_ids={"raspi":1, "herkulex":2, "base roulante":3}
reversed_tab_ids=dict((v,k) for (k,v) in tab_ids.items())

# l'en tête est un int
def decomposer_en_tete(en_tete):
    return (en_tete>>8,(en_tete>>4)&15,en_tete&15)

class CanRx(Node):
    def __init__(self):

        super().__init__('can_rx')
        # Subscribe to all data on the can_robot bus

        self.bus = can.interface.Bus(interface='socketcan',
                      channel='can0',
                      receive_own_messages=True)

        notifier = can.Notifier(self.bus, [self.on_message])
        self.imu_data_publisher = self.create_publisher(Imu, 'imu_data', 10)
        self.motors_feedback = self.create_publisher(JointState, 'joint_state', 10)
        self.arm_feedback_publisher = self.create_publisher(ArmFeedback, 'arm_feedback', 10)
        self.ultrasound_publisher = self.create_publisher(Range, 'range', 10)
        # self.can_rx_service = self.create_service(CanRx, 'can_rx', self.send_message)

        # Create a publisher for motorsFeedback
        # self.motors_feedback_publisher = self.create_publisher(MotorsFeedback, 'motors_feedback', 10)

    def send_message(self, request, response):
        """
        Send a message to the can_robot bus
        """
        # Create a message
        msg = can.Message(arbitration_id=0x123, data=[0, 25, 0, 1, 3, 1, 4, 1], is_extended_id=False)
        # Send the message
        self.bus.send(msg)
        # Return the response
        response.success = True
        return response

    def on_message(self, msg):
        """
        Callback function for the can_robot bus
        """
        (prio, id_dest, id_or) = decomposer_en_tete(msg.arbitration_id)
        print("message reçu de " + reversed_tab_ids[id_or] + " : " + msg.data.decode() + " à destination de " + reversed_tab_ids[id_dest])
        # Publish the message to the appropriate topic
        # self.can_rx_publisher.publish(msg)
        self.get_logger().info('Received message: {0}'.format(msg))
    def init_communication(self):
        """
        Initialize the communication with the can_robot bus
        """
        pass

    def receive_data(self):
        """
        Receive data from the can_robot bus
        1. Check IDs
        2. Extract data
        3. Publish data to the appropriate topic
        """
        (prio, id_dest, id_or) = decomposer_en_tete(self.arbitration_id)
        data = self.data

        match id_or:
            case 1:
                #msg comes from raspi -> loopback?
                pass
            case 2:
                #msg comes from herkulex, data is published in ArmFeedback topic
                #prepare message and convert data into usable units

                arm_msg = ArmFeedback()
                #arm_msg.id = data[0]
                #arm_msg.plier_open =
                #arm_msg.position =
                #arm_msg.speed =
                #arm_msg.err_flag =
                #arm_msg.rtr_flag =
                #arm_msg.eff_flag =
                self.arm_feedback_publisher.publish(arm_msg)
            case 3:
                # msg comes from motors, data is published in MotorsFeedback topic
                motors_msg = JointState()
                match data[0]:
                    case 0:
                        motors_msg.name = "front"
                    case 1:
                        motors_msg.name = "back"
                    case 2:
                        motors_msg.name = "left"
                    case 1:
                        motors_msg.name = "right"
                motors_msg.position = ()
                motors_msg.velocity = ()
                motors_msg.effort = 0
                self.motors_feedback_publisher.publish(motors_msg)
                pass
            case _:
                #self.can_rx_publisher.publish(self)
                pass

        pass

    def close_communication(self):
        """
        Close the communication with the can_robot bus
        """
        pass


def main(args=None):
    rclpy.init(args=args)

    node = CanRx()

    rclpy.spin(node)

    r = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            r.sleep()
    except KeyboardInterrupt:
        bus.shutdown()


    rclpy.shutdown()


if __name__ == '__main__':
    main()
