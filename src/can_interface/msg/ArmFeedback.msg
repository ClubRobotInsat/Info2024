#Action type enum
uint8 MOVE_TO_HOME_ACTION=1
uint8 MOVE_TO_READY_ACTION=2
uint8 PUT_OBJECT_STOCK_ACTION=3
uint8 GET_OBJECT_STOCK_ACTION=4
uint8 GRAB_ACTION=5
uint8 RELEASE_ACTION=6

uint8 action_id
bool action_result


## ArmFeedback.msg
# @brief A message containing Arm's data.
# action_id The arm's action's ID
# action_result True if action succeeded, false otherwise