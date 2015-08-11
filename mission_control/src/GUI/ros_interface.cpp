#include "ros_interface.hpp"
#include "ros/service_client.h"
#include "mission_control/function_defines.h"
#include "mission_control/ui_api_defines.h"

using namespace std;

void RosInterface::run()
{
    //initialize ros rubscribers
    hw_state_sub = n.subscribe(UIAPI_HW_STATES, 10, &RosInterface::hwStateCB, this);
    exec_progress_sub = n.subscribe(UIAPI_EXEC_PROGRESS, 10, &RosInterface::execProgressCB, this);
    instr_progress_sub = n.subscribe(UIAPI_INSTR_PROGRESS, 10, &RosInterface::instrProgressCB, this);

    ros::spin();
}

void RosInterface::hwStateCB(const mission_control::HardwareStates::ConstPtr &msg)
{
    mission_control::HardwareStates states;

    for(int i=0;i<(int)msg->hardware_states.size();i++)
    {
        states.hardware_states.push_back(msg->hardware_states[i]);
    }

    emit hw_states_received(states);
}

void RosInterface::execProgressCB(const mission_control::Progress::ConstPtr &msg)
{
    emit exec_progress_update_received(msg);
}

void RosInterface::instrProgressCB(const mission_control::Progress::ConstPtr &msg)
{
    emit instr_progress_update_received(msg);
}
