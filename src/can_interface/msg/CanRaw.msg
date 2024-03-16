int32 arbitration_id
uint8[8] data
uint8 err_flag
uint8 rtr_flag
uint8 eff_flag

## CanRaw.msg
# @brief A message containing raw CAN data.
# @param arbitration_id The CAN ID of the message
# @param data The raw data of the message
# @param err_flag The error flag of the message (0 = data frame, 1 = error message)
# @param rtr_flag The RTR flag of the message (0 = data frame, 1 = remote frame)
# @param eff_flag The EFF flag of the message (0 = standard frame 11 bits, 1 = extended frame 29 bits)