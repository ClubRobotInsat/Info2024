uint8 dest
uint8 command_id
uint8 servo_id
float32 angle
float32 speed
uint8 mode
uint8 torque
uint16 duration

## ServoCmd.msg
# @brief A message containing information to control a servomotor.
# @param dest The id of the desired receiver
# @param command id 
# @param servo_id The ID of the servo
# @param angle The servo's angle
# @param speed The servo's rotation speed [Units?]
# @param mode sets the servo's mode (0 for Position Control, 1 for Speed Control)
# @param torque sets the servo's torque parameter to a value between 0 and 2
