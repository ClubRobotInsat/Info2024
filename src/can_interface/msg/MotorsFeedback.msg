uint8 motor_id
float32 position
float32 velocity
uint8 err_flag
uint8 rtr_flag
uint8 eff_flag


## MotorsFeedback.msg
# @brief A message containing Motors' data.
# @param motor_id The motor's ID (0 = front, 1 = back, 2 = left, 3 = right)
# @param position The position of the motor (angle in degrees)
# @param speed The speed of the engine (in degrees / second)
# @param err_flag The error flag of the message (0 = data frame, 1 = error message)
# @param rtr_flag The RTR flag of the message (0 = data frame, 1 = remote frame)
# @param eff_flag The EFF flag of the message (0 = standard frame 11 bits, 1 = extended frame 29 bits)