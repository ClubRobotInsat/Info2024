uint8 dest
uint8 command_id
uint8 imu_id
uint8 tof_id

## SensorCmd.msg
# @brief A message to request sensor values.
# @param dest The id of the desired receiver
# @param command_id The ID of the command 
# @param imu_id The ID of the inertial measurement unit 
# @param tof_id The ID of the Time-Of-Flight sensor