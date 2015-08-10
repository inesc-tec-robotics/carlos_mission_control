
#ifndef CARLOS_MISSION_CTRL_DEFINES_H
#define CARLOS_MISSION_CTRL_DEFINES_H

///NAMESPACE DEFINITIONS//////////
/* Params */
#define CARLOS_FSM_FREQUENCY  "/carlos/fsm_frequency"

/* Messages */
#define CARLOS_BASE_STATE_MSG "/carlos/base_state"
#define CARLOS_ARM_STATE_MSG  "/carlos/arm_state"
#define CARLOS_STUDS_POS_MSG  "/carlos/studs_pose"
#define CARLOS_STUDS_PRJ_MSG  "/carlos/studs_projector"

/* Services */

/* Actions */
#define CARLOS_WELD_ACTION         "/carlos/execute_weld"
#define CARLOS_DISTRIBUTION_ACTION "/carlos/generate_distribution"
#define CARLOS_MOVE_ACTION         "/carlos/move_platform"
#define CARLOS_MOVE_ARM_ACTION     "/carlos/move_arm"
#define CARLOS_PROJECTION_ACTION   "/carlos/projection_pose"
#define CARLOS_TEACHING_ACTION     "/carlos/perform_teaching"
/////////////////////////////////

/* Parameters */
#define DEFAULT_STATE_FREQ 2.0

namespace hardware
{
enum states
{
    IDLE = 0,           //Hardware system idle (operational and ready)
    BUSY,               //Hardware is busy (working)
    ERROR,              //Hardware is in error (could be either functional error or emergency stop)
    NOT_CONNECTED       //Hardware component not connected (this state is used by mission controller as a pseudo state for hardware components until connection is established)
};
}

namespace carlos_system
{
enum states
{
    IDLE = 0,
    EXECUTING,
    INSTRUCTING,
    ASSISTING,
    INITIALIZING
};
}

namespace mission
{
enum states
{
    EMPTY = 0,
    PARTIALLY_CONFIGURED,
    CONFIGURED,
    PARTIALLY_INSTRUCTED,
    INSTRUCTED,
    PARTIALLY_COMPLETED,
    COMPLETED
};
}

namespace stud
{
enum states
{
    PENDING,
    SUCCEEDED,
    FAILED
};
}

namespace vision
{
  enum states
  {
    STOPPED = 0,
    DETECT,
    TRACK
  };
}

#endif /* CARLOS_MISSION_CTRL_DEFINES_H */
