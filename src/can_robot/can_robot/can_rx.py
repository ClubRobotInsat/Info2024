#!/usr/bin/env python3

import rclpy
import struct

from rclpy.node import Node

from sensor_msgs.msg import Imu, JointState, Range
from geometry_msgs.msg import Quaternion, Vector3
from can_interface.msg import ArmFeedback, CanRaw, Tirette

##########################################################################################
# HEADER IDs
tab_ids = {"urgence": 0, "raspi": 1, "base_roulante1": 2, "base-roulante2": 3, "herkulex": 4, "capteurs": 5}
reversed_tab_ids = dict((v, k) for (k, v) in tab_ids.items())

# HERKULEX INSTRUCTION IDs
arm_instr_id_tab = {"stop": 0, "ping": 1,
                    "setAngle": 2, "getAngle": 3, "getAngleACK": 4, "setSpeed": 5, "getSpeed": 6, "getSpeedACK": 7,
                    "setSpinDuration": 8, "changeMode": 9, "getMode": 10, "setTorque": 11, "getTorque": 12,
                    "reboot": 13, "clearError": 14, "getError": 15, "getStatusDetail": 16,
                    "grab": 17, "release": 18, "moveToHomePosition": 19, "moveToReadyPosition": 20,
                    "moveFindObjetLow": 21, "moveFindObjetHigh": 22,
                    "putInStock": 23, "getFromStock": 24, "movePlaceObject": 25, "nextRing": 26, "previousRing": 27}
arm_instr_dict = dict((v, k) for (k, v) in arm_instr_id_tab.items())

# BASE_ROULANTE INSTRUCTION IDs
motor_instr_id_tab = {"stop": 0, "ping": 1, "start": 2,
                      "setTargetSpeed": 3, "getTargetSpeedACK": 4, "setMotorDirection": 5, "getCurrentSpeed": 6,
                      "getMotorDirection": 7}
motor_instr_dict = dict((v, k) for (k, v) in motor_instr_id_tab.items())

# CAPTEURS INSTRUCTION IDs
sensor_instr_id_tab = {"stop": 0, "ping": 1,
                       "get_accel": 2, "get_angle_vel": 3, "get_distance": 4,
                       "tirette": 5}
sensor_instr_dict = dict((v, k) for (k, v) in sensor_instr_id_tab.items())


# l'en tête est un int
def decomposer_en_tete(en_tete):
    return en_tete >> 8, (en_tete >> 4) & 15, en_tete & 15


class CanRx(Node):
    def __init__(self):

        super().__init__('can_rx')

        # self.bus = can.interface.Bus(interface='socketcan',
        #              channel='can0',
        #              receive_own_messages=True)

        # notifier = can.Notifier(self.bus, [self.on_message])
        self.canraw_subscriber = self.create_subscription(CanRaw, 'can_raw_rx', self.receive_data, 10)
        # Publish in all data feedback topics
        self.imu_data_publisher = self.create_publisher(Imu, 'imu_data', 10)
        self.motors_feedback_publisher = self.create_publisher(JointState, 'joint_state', 10)
        self.arm_feedback_publisher = self.create_publisher(ArmFeedback, 'arm_feedback', 10)
        self.ultrasound_publisher = self.create_publisher(Range, 'range', 10)
        self.tirette_publisher = self.create_publisher(Tirette, 'tirette', 10)

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
                case "urgence":
                    # emergency msg -> where to propagate?
                    pass
                case "raspi":
                    # msg comes from raspi -> loopback?
                    pass
                case "herkulex":
                    arm_msg = ArmFeedback()  # init arm message
                    arm_instruction = arm_instr_dict.get(data[0])  # identify instruction
                    match arm_instruction:
                        case "grab":
                            arm_msg.action_id = ArmFeedback.GRAB_ACTION
                        case "release":
                            arm_msg.action_id = ArmFeedback.RELEASE_ACTION
                        case "moveToHomePosition":
                            arm_msg.action_id = ArmFeedback.MOVE_TO_HOME_ACTION
                        case "moveToReadyPosition":
                            arm_msg.action_id = ArmFeedback.MOVE_TO_READY_ACTION
                        case "moveFindObjetLow":
                            arm_msg.action_id = ArmFeedback.FIND_LOW_ACTION
                        case "moveFindObjetHigh":
                            arm_msg.action_id = ArmFeedback.FIND_HIGH_ACTION
                        case "putInStock":
                            arm_msg.action_id = ArmFeedback.PUT_OBJECT_STOCK_ACTION
                        case "getFromStock":
                            arm_msg.action_id = ArmFeedback.GET_OBJECT_STOCK_ACTION
                        case "movePlaceObject":
                            arm_msg.action_id = ArmFeedback.PLACE_OBJECT_ACTION
                        case "nextRing":
                            arm_msg.action_id = ArmFeedback.NEXT_RING_ACTION
                        case "previousRing":
                            arm_msg.action_id = ArmFeedback.PREVIOUS_RING_ACTION
                        case _:
                            pass
                    arm_msg.action_result = bool(data[1])  # get action result

                    self.arm_feedback_publisher.publish(arm_msg)  # publish in Arm Feedback topic
                    self.get_logger().info('Sent message: {0}'.format(arm_msg))  # display

                case "base_roulante1":
                    motors_msg = JointState()  # init Motor message
                    motors_msg.name = []
                    motors_msg.position = []
                    motors_msg.velocity = []

                    motor_instruction = motor_instr_dict.get(data[0])  # identify instruction
                    match motor_instruction:
                        case "getTargetSpeedACK":
                            if (data[3] != 0):  # if speed is Ack
                                motors_msg.name.append(str(data[1]))  # append motor ID
                                motors_msg.velocity.append(convert_mm_into_m(data[2]))  # append speed in m/s
                            else:
                                pass
                        case "getCurrentSpeed":
                            motors_msg.name.append(str(data[1]))
                            motors_msg.velocity.append(
                                convert_mm_into_m(convert_4bytes_into_signedfloat(data[2], data[3], data[4], data[5])))
                        case _:
                            pass

                    self.motors_feedback_publisher.publish(motors_msg)  # publish message in Motor Feedback topic
                    self.get_logger().info('Sent message: {0}'.format(motors_msg))  # display

                case "base_roulante2":
                    motors_msg = JointState()  # init Motor message
                    motors_msg.name = []
                    motors_msg.position = []
                    motors_msg.velocity = []

                    motor_instruction = motor_instr_dict.get(data[0])  # identify instruction
                    match motor_instruction:
                        case "getTargetSpeedACK":
                            if (data[3] != 0):  # if speed is Ack
                                motors_msg.name.append(str(data[1]))  # append motor ID
                                motors_msg.velocity.append(convert_mm_into_m(data[2]))  # append speed in m/s
                            else:
                                pass
                        case "getCurrentSpeed":
                            motors_msg.name.append(str(data[1]))
                            motors_msg.velocity.append(
                                convert_mm_into_m(convert_4bytes_into_signedfloat(data[2], data[3], data[4], data[5])))
                        case _:
                            pass

                    self.motors_feedback_publisher.publish(motors_msg)  # publish message in Motor Feedback topic
                    self.get_logger().info('Sent message: {0}'.format(motors_msg))  # display

                case "capteurs":
                    sensor_instruction = sensor_instr_dict.get(data[0])  # identify instruction
                    match sensor_instruction:
                        case "get_accel":
                            imu_msg = Imu()  # init imu message
                            imu_msg.orientation_covariance = [0.0] * 9  # covariance of measurements is not known
                            imu_msg.angular_velocity_covariance = [0.0] * 9
                            imu_msg.linear_acceleration_covariance = [0.0] * 9
                            vec = Vector3()
                            vec.x = convert_signedfloat_into_g(convert_2bytes_into_signedfloat(data[2], data[
                                3]))  # convert 16 bits into signed float in unit g
                            vec.y = convert_signedfloat_into_g(convert_2bytes_into_signedfloat(data[4], data[5]))
                            vec.z = convert_signedfloat_into_g(convert_2bytes_into_signedfloat(data[6], data[7]))
                            imu_msg.linear_acceleration = vec
                            self.imu_data_publisher.publish(imu_msg)  # publish in Imu Data topic
                            self.get_logger().info('Sent message: {0}'.format(imu_msg))  # display
                        case "get_angle_vel":
                            imu_msg = Imu()  # init imu message
                            imu_msg.orientation_covariance = [0.0] * 9  # covariance of measurements is not known
                            imu_msg.angular_velocity_covariance = [0.0] * 9
                            imu_msg.linear_acceleration_covariance = [0.0] * 9
                            vec = Vector3()
                            vec.x = convert_signedfloat_into_deg_s(convert_2bytes_into_signedfloat(data[2], data[
                                3]))  # convert 16 bits into signed float in unit deg/s
                            vec.y = convert_signedfloat_into_deg_s(convert_2bytes_into_signedfloat(data[4], data[5]))
                            vec.z = convert_signedfloat_into_deg_s(convert_2bytes_into_signedfloat(data[6], data[7]))
                            imu_msg.angular_velocity = vec
                            self.imu_data_publisher.publish(imu_msg)  # publish in Imu Data topic
                            self.get_logger().info('Sent message: {0}'.format(imu_msg))  # display
                        case "get_distance":
                            range_msg = Range()  # init ultrasound message
                            range_msg.range = convert_mm_into_m(
                                data[1])  # assuming distance measure is stored in data[1]
                            self.ultrasound_publisher.publish(range_msg)  # publish in Ultrasound topic
                            self.get_logger().info('Sent message: {0}'.format(range_msg))  # display
                        case "tirette":
                            tirette_msg = Tirette()  # init tirette message
                            tirette_msg.go = bool(data[1])  # get tirette state
                            self.tirette_publisher.publish(tirette_msg)  # publish in Tirette topic
                            self.get_logger().info('Sent message: {0}'.format(tirette_msg))  # display
                        case _:
                            pass
                case _:
                    pass


def convert_4bytes_into_signedfloat(b1, b2, b3, b4):
    '''
    Convert 4 bytes into a signed float value
    Big endian
    '''
    int_value = b4 | (b3 << 8) | (b2 << 16) | (b1 << 24)
    return struct.unpack('f', struct.pack('I', int_value))[0]


def convert_2bytes_into_signedfloat(b1, b2):
    '''
    Convert 2 bytes into a signed float value
    Big endian
    '''
    int_value = b2 | (b1 << 8)
    return struct.unpack('f', struct.pack('I', int_value))[0]


def convert_mm_into_m(nb):
    '''
    Convert input nb from mm into m
    :param nb in mm
    :return: nb in m
    '''
    return nb / 1000


def convert_signedfloat_into_g(fl):
    '''
    For any reclamation: ask LIAM
    '''
    conversion_value = 4.79e-3
    return fl * conversion_value


def convert_signedfloat_into_deg_s(fl):
    '''
    For any reclamation: ask LIAM
    '''
    conversion_value = 6.098e-2
    return fl * conversion_value


def main(args=None):
    rclpy.init(args=args)

    node = CanRx()

    rclpy.spin(node)

    r = rospy.Rate(10)  # 10hz
    try:
        while not rospy.is_shutdown():
            r.sleep()
    except KeyboardInterrupt:
        bus.shutdown()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
