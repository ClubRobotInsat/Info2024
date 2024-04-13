uint8 prio
uint8 dest
uint8 command_id
uint8 motor_id
uint8 direction
float32 speed 
uint8 extra

## MotorCmd.msg
# @brief A message containing information to control a motor.
# @param prio The priority of the message
# @param dest The id of the desired receiver
# @param command_id The ID of the command 
# @param motor_id The ID of the motor
# @param direction Direction of the motor's rotation (0 for clockwise, 1 for counter clockwise)
# @param speed in mm/s
# @param extra extra byte just in case
