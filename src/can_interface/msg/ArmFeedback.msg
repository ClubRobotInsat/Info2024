uint8 id
bool plier_open
float32 position  # Position actuelle de l'actuateur (en radians ou degrés)
float32 speed
uint8 err_flag
uint8 rtr_flag
uint8 eff_flag

## ArmFeedback.msg
# @brief A message containing Arm's data.
# @param plier_open True if the plier is open, false if closed
# @param id The ID of the Herkulex sending the message (0 = plier, 1 = herk. closest to plant, 2 = second closest, 3 = third closest)
# @param position The position of Herkulex (angle in degrees)
# @param speed The speed of Herkulex (in degrees / second)
# @param err_flag The error flag of the message (0 = data frame, 1 = error message)
# @param rtr_flag The RTR flag of the message (0 = data frame, 1 = remote frame)
# @param eff_flag The EFF flag of the message (0 = standard frame 11 bits, 1 = extended frame 29 bits)