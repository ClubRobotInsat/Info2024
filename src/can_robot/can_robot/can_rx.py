#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
import can

from sensor_msgs.msg import Imu, JointState, Range
from can_interface.msg import ArmFeedback, CanRaw




##########################################################################################
tab_ids={"raspi":1, "herkulex":2, "base_roulante":3}
reversed_tab_ids=dict((v,k) for (k,v) in tab_ids.items())

# l'en tête est un int
def decomposer_en_tete(en_tete):
    return (en_tete>>8,(en_tete>>4)&15,en_tete&15)

class CanRx(Node):
    def __init__(self):

        super().__init__('can_rx')
        # Subscribe to all data on the can_robot bus

        #self.bus = can.interface.Bus(interface='socketcan',
        #              channel='can0',
        #              receive_own_messages=True)

        #notifier = can.Notifier(self.bus, [self.on_message])
        self.canraw_subscriber = self.create_subscription(CanRaw, 'can_raw_rx', self.receive_data, 10)
        self.imu_data_publisher = self.create_publisher(Imu, 'imu_data', 10)
        self.motors_feedback_publisher = self.create_publisher(JointState, 'joint_state', 10)
        self.arm_feedback_publisher = self.create_publisher(ArmFeedback, 'arm_feedback', 10)
        self.ultrasound_publisher = self.create_publisher(Range, 'range', 10)


    def receive_data(self, msg):
        """
        Receive data from the can_robot bus
        1. Check IDs
        2. Extract data
        3. Publish data to the appropriate topic
        """
        (prio, id_dest, id_or) = decomposer_en_tete(msg.arbitration_id)
        source_id = reversed_tab_ids.get(id_or)
        data = msg.data

        if msg.err_flag:
            self.get_logger().error("CAN MSG ERROR")
            self.get_logger().error("{0}".format(msg))

        else:
            self.get_logger().info("Received Dataframe:")
            self.get_logger().info("{0}".format(msg))

            match source_id:
                case "raspi":
                    #msg comes from raspi -> loopback?
                    pass
                case "herkulex":
                    #msg comes from herkulex, data is published in ArmFeedback topic
                    arm_msg = ArmFeedback()
                    arm_msg.name = []
                    arm_msg.position = []
                    arm_msg.speed = []

                    arm_instr_id_tab={"stop":0,"ping":1, "setAngle":2, "getAngle":3, "getAngleACK":4, "setSpeed":5, "getSpeed":6, "getSpeedACK":7,
                                      "setSpinDuration":8, "changeMode":9, "getMode":10, "setTorque":11, "getTorque":12, "reboot":13,
                                      "clearError":14, "getError":15, "getStatusDetail":16, "grab":17, "release":18}
                    arm_instr_dict=dict((v,k) for (k,v) in arm_instr_id_tab.items())
                    # identify instruction
                    arm_instruction=arm_instr_dict.get(data[0])
                    match arm_instruction:
                        case "getAngle":
                            arm_msg.name.append(str(data[1]))
                            arm_msg.position.append(data[2])
                        case "getAngleACK":
                            if data[3]:
                                arm_msg.name.append(str(data[1]))
                                arm_msg.position.append(data[2])
                        case "getSpeed":
                            arm_msg.name.append(str(data[1]))
                            arm_msg.speed.append(data[2])
                        case "getSpeedACK": #TODO: convert in rad/s
                            if data[3]:
                                arm_msg.name.append(str(data[1]))
                                arm_msg.speed.append(data[2])
                        case _:
                            pass
                    pass
                    #arm_msg.plier_open = #TODO update instruction list for plier instructions
                    self.arm_feedback_publisher.publish(arm_msg)
                    #display message sent to subscribers
                    self.get_logger().info('Sent message: {0}'.format(arm_msg))

                case "base_roulante":
                    # msg comes from motors, data is published in MotorsFeedback topic
                    motors_msg = JointState()
                    motors_msg.name = []
                    motors_msg.position = []
                    motors_msg.velocity = []
                    motors_msg.effort = []

                    motor_instr_id_tab={"stop":0,"ping":1, "setTargetSpeed":2, "getTargetSpeedACK":3, "setMotorDirection":4, "getCurrentSpeed":5, "getMotorDirection":6}
                    motor_instr_dict=dict((v,k) for (k,v) in motor_instr_id_tab.items())
                    # identify instruction
                    motor_instruction = motor_instr_dict.get(data[0])
                    match motor_instruction:
                        case "getTargetSpeedACK":
                            if (data[3]!=0):
                                motors_msg.name.append(str(data[1]))
                                motors_msg.velocity.append(data[2])
                            else:
                                pass
                        case "getCurrentSpeed":
                            motors_msg.name.append(str(data[1]))
                            motors_msg.velocity.append(data[2])
                        case "getMotorDirection":
                            motors_msg.name.append(str(data[1]))
                            motors_msg.position.append(data[2])
                        case _:
                            pass

                    self.motors_feedback_publisher.publish(motors_msg)
                    #display message sent to subscribers
                    self.get_logger().info('Sent message: {0}'.format(motors_msg))

                case _:
                    pass

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
