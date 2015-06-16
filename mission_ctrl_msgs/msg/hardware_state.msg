#This msg is used for publishing the state for each hardware subsystem (for the mission controller to receive)

uint8 state		#maps to the enum "HARDWARE::STATES" in "mission_ctrl_msgs" file: "states.h"
string description	#additional information about the state (for GUI/operator information purposes)
