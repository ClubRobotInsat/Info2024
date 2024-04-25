string[] name
float32[] position  # Position actuelle de l'actuateur (en radians ou degrés)
float32[] speed
bool plier_open

## ArmFeedback.msg
# @brief A message containing Arm's data.
# @param name an ID array of the Herkulex whithin the arm (0 = herk. closest to plant, 1 = second closest, 2 = third closest)
# @param position The position of each Herkulex (angle in degrees)
# @param speed The speed of each Herkulex (in degrees / second)
# @param plier_open True if the plier is open, false if closed
