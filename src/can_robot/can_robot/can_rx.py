#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
import can

from sensor_msgs.msg import Imu, JointState, Range
from geometry_msgs.msg import Quaternion, Vector3
from can_interface.msg import ArmFeedback, CanRaw




##########################################################################################
#HEADER IDs
tab_ids={"urgence":0, "raspi":1, "herkulex":2, "base_roulante":3, "capteurs":4}
reversed_tab_ids=dict((v,k) for (k,v) in tab_ids.items())

#HERKULEX INSTRUCTION IDs
arm_instr_id_tab={"stop":0,"ping":1,
                  "setAngle":2, "getAngle":3, "getAngleACK":4, "setSpeed":5, "getSpeed":6, "getSpeedACK":7, "setSpinDuration":8, "changeMode":9, "getMode":10, "setTorque":11, "getTorque":12,
                  "reboot":13, "clearError":14, "getError":15, "getStatusDetail":16,
                  "grab":17, "release":18, "moveToHomePosition":19, "moveToReadyPosition":20, "putInStock":21, "getFromStock":22}
arm_instr_dict=dict((v,k) for (k,v) in arm_instr_id_tab.items())

#BASE_ROULANTE INSTRUCTION IDs
motor_instr_id_tab={"stop":0,"ping":1, "start":2,
                    "setTargetSpeed":3, "getTargetSpeedACK":4, "setMotorDirection":5, "getCurrentSpeed":6, "getMotorDirection":7}
motor_instr_dict=dict((v,k) for (k,v) in motor_instr_id_tab.items())

#CAPTEURS INSTRUCTION IDs
sensor_instr_id_tab={"stop":0,"ping":1,
                     "getOrientationImu":2, "getSpeedImu":3, "getAccelerationImu":4,
                     "getFieldUltrasound":5, "getRangeUltrasound":6}
sensor_instr_dict=dict((v,k) for (k,v) in sensor_instr_id_tab.items())


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
                case "urgence":
                    #emergency msg -> where to propagate?
                    pass
                case "raspi":
                    #msg comes from raspi -> loopback?
                    pass
                case "herkulex":
                    arm_msg = ArmFeedback() #init arm message
                    arm_instruction=arm_instr_dict.get(data[0]) #identify instruction
                    match arm_instruction:
                        case "grab":
                           arm_msg.action_id = ArmFeedback.GRAB_ACTION
                        case "release":
                            arm_msg.action_id = ArmFeedback.RELEASE_ACTION
                        case "moveToHomePosition":
                            arm_msg.action_id = ArmFeedback.MOVE_TO_HOME_ACTION
                        case "moveToReadyPosition":
                            arm_msg.action_id = ArmFeedback.MOVE_TO_READY_ACTION
                        case "putInStock":
                            arm_msg.action_id = ArmFeedback.PUT_OBJECT_STOCK_ACTION
                        case "getFromStock":
                            arm_msg.action_id = ArmFeedback.GET_OBJECT_STOCK_ACTION
                        case _:
                            pass
                    arm_msg.action_result = bool(data[1]) #get action result

                    self.arm_feedback_publisher.publish(arm_msg) #publish in Arm Feedback topic
                    self.get_logger().info('Sent message: {0}'.format(arm_msg)) #display

                case "base_roulante":
                    motors_msg = JointState() #init Motor message
                    motors_msg.name = []
                    motors_msg.position = []
                    motors_msg.velocity = []
                    motors_msg.effort = []

                    motor_instruction = motor_instr_dict.get(data[0]) # identify instruction
                    match motor_instruction:
                        case "getTargetSpeedACK":
                            if (data[3]!=0): #if speed is Ack
                                motors_msg.name.append(str(data[1])) #append motor ID
                                motors_msg.velocity.append(convert_speed_into_ms(data[2])) #append speed in m/s
                            else:
                                pass
                        case "getCurrentSpeed":
                            motors_msg.name.append(str(data[1]))
                            motors_msg.velocity.append(convert_speed_into_ms(data[2]))
                        case "getMotorDirection":
                            motors_msg.name.append(str(data[1])) #TODO check direction of motor and ???
                            motors_msg.position.append(data[2])
                        case _:
                            pass

                    self.motors_feedback_publisher.publish(motors_msg) #publish message in Motor Feedback topic
                    self.get_logger().info('Sent message: {0}'.format(motors_msg)) #display

                case "capteurs":
                    imu_msg = Imu() #init imu message
                    range_msg = Range() #init ultrasound message

                    sensor_instruction = sensor_instr_dict.get(data[0]) #identify instruction
                    match sensor_instruction:
                        case "getOrientationImu":
                            quat = Quaternion()
                            #quat.x = data[]
                            #quat.y = data[]
                            #quat.z = data[]
                            #quat.w = data[]
                            imu_msg.orientation = quat
                            self.imu_data_publisher.publish(imu_msg) #publish in Imu Data topic
                            self.get_logger().info('Sent message: {0}'.format(imu_msg)) #display
                        case "getSpeedImu":
                            vec = Vector3()
                            #vec.x = data[]
                            #vec.y = data[]
                            #vec.z = data[]
                            imu_msg.angular_velocity = vec
                            self.imu_data_publisher.publish(imu_msg) #publish in Imu Data topic
                            self.get_logger().info('Sent message: {0}'.format(imu_msg)) #display
                        case "getAccelerationImu":
                            vec = Vector3()
                            #vec.x = data[]
                            #vec.y = data[]
                            #vec.z = data[]
                            imu_msg.linear_acceleration = vec
                            self.imu_data_publisher.publish(imu_msg) #publish in Imu Data topic
                            self.get_logger().info('Sent message: {0}'.format(imu_msg)) #display
                        case "getFieldUltrasound":
                            range_msg.radiation_type = 0
                            #range_msg.field_of_view = data[] #ITS A FLOAT VALUE HENCE WTF
                            self.ultrasound_publisher.publish(range_msg) #publish in ultrasound data topic
                            self.get_logger().info('Sent message: {0}'.format(range_msg)) #display
                        case "getRangeUltrasound":
                            range_msg.radiation_type = 0
                            #range_msg.range = data[] #ITS A FLOAT VALUE HENCE WTF
                            self.ultrasound_publisher.publish(range_msg) #publish in ultrasound data topic
                            self.get_logger().info('Sent message: {0}'.format(range_msg)) #display
                        case _:
                            pass
                case _:
                    pass


def convert_speed_into_ms(speed):
    '''
    Convert input speed from mm/sec into m/sec
    :param speed in mm/sec
    :return: speed in m/sec
    '''
    return speed/1000


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
