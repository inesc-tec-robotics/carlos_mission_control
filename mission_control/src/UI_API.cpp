#include "ros/node_handle.h"
#include "mission_handler.hpp"
#include "UI_API.hpp"
#include "hw_state_machine.hpp"
#include "mission_control/HardwareStates.h"
#include "system_engine.hpp"
#include "execution_engine.hpp"
#include "instruction_engine.hpp"
#include "mission_control/ui_api_defines.h"

#define PUB_FREQ 10.0 //publication frequency in Hz

using namespace std;

UiAPI* UiAPI::instance_ = NULL;

UiAPI* UiAPI::getInstance()
{
    if (!instance_)   // Only allow one instance of class to be generated.
        instance_ = new UiAPI();

    return instance_;
}

UiAPI::UiAPI()
{
    load_mission_srv_ = n.advertiseService(UIAPI_LOAD_MISSION, &UiAPI::loadMissionCB, this);
    save_mission_srv_ = n.advertiseService(UIAPI_SAVE_MISSION, &UiAPI::saveMissionCB, this);
    save_mission_as_srv_ = n.advertiseService(UIAPI_SAVE_MISSION_AS, &UiAPI::saveMissionAsCB, this);
    create_new_mission_srv_ = n.advertiseService(UIAPI_CREATE_NEW_MISSION, &UiAPI::createNewMissionCB, this);
    get_mission_meta_srv_ = n.advertiseService(UIAPI_GET_MISSION_META, &UiAPI::getMissionMetaCB, this);
    get_task_list_srv_ = n.advertiseService(UIAPI_GET_TASK_LIST, &UiAPI::getTaskListCB, this);
    get_mission_list_srv_ = n.advertiseService(UIAPI_GET_MISSION_LIST, &UiAPI::getMissionListCB, this);
    exec_start_srv_ = n.advertiseService(UIAPI_EXEC_START, &UiAPI::execStartCB, this);
    exec_abort_srv_ = n.advertiseService(UIAPI_EXEC_ABORT, &UiAPI::execAbortCB, this);
    exec_pause_srv_ = n.advertiseService(UIAPI_EXEC_PAUSE_RESUME, &UiAPI::execPauseCB, this);
    exec_skip_stud_srv_ = n.advertiseService(UIAPI_EXEC_SKIP_STUD, &UiAPI::execSkipStudCB, this);
    exec_skip_task_srv_ = n.advertiseService(UIAPI_EXEC_SKIP_TASK, &UiAPI::execSkipTaskCB, this);
    exec_retry_srv_ = n.advertiseService(UIAPI_EXEC_RETRY, &UiAPI::execRetryCB, this);
    instr_start_srv_ = n.advertiseService(UIAPI_INSTR_START, &UiAPI::instrStartCB, this);
    instr_abort_srv_ = n.advertiseService(UIAPI_INSTR_ABORT, &UiAPI::instrAbortCB, this);
    instr_pause_srv_ = n.advertiseService(UIAPI_INSTR_PAUSE_RESUME, &UiAPI::instrPauseCB, this);
    instr_skip_task_srv_ = n.advertiseService(UIAPI_INSTR_SKIP_TASK, &UiAPI::instrSkipTaskCB, this);
    instr_retry_srv_ = n.advertiseService(UIAPI_INSTR_RETRY, &UiAPI::instrRetryCB, this);

    ROS_INFO("ROS ServiceServers for UI API started");

    //publisher for execution progress:
    exec_progress_pub_ = n.advertise<mission_control::Progress>(UIAPI_EXEC_PROGRESS,10, true);

    //publisher for instruction progress:
    instr_progress_pub_ = n.advertise<mission_control::Progress>(UIAPI_INSTR_PROGRESS,10, true);

    //create publisher of states:
    state_pub_ = n.advertise<mission_control::HardwareStates>(UIAPI_HW_STATES,10);

    //create timer for state publishing:
    state_pub_timer_ = n.createTimer(ros::Duration((1/PUB_FREQ)), &UiAPI::statePubTimeout, this);
}

UiAPI::~UiAPI()
{
}

void UiAPI::execProgressUpdate(mission_control::Progress &progress)
{
    exec_progress_pub_.publish(progress);
}

void UiAPI::instrProgressUpdate(mission_control::Progress &progress)
{
    instr_progress_pub_.publish(progress);
}

bool UiAPI::getTaskListCB(mission_control::getTaskList::Request &request, mission_control::getTaskList::Response &response)
{
    ROS_DEBUG("getTaskListCB");
    if(!MissionHandler::getInstance()->isLoaded())
    {
        return false;
    }

    vector<string> tasks = MissionHandler::getInstance()->getTaskList();

    response.tasks = tasks;

    return true;
}

bool UiAPI::getMissionListCB(mission_control::getMissionList::Request &request, mission_control::getMissionList::Response &response)
{
    ROS_DEBUG("getMissionListCB");
    vector<string> missions = MissionHandler::getInstance()->getListOfMissions();

    response.missions = missions;

    return true;
}

bool UiAPI::loadMissionCB(mission_control::loadMission::Request &request, mission_control::loadMission::Response &response)
{
    ROS_DEBUG_STREAM("loadMissionCB - name: " << request.name);
    if(!MissionHandler::getInstance()->isMission(request.name))
        return false;

    if(!MissionHandler::getInstance()->load(request.name))
        return false;

    return true;
}

bool UiAPI::saveMissionCB(mission_control::saveMission::Request &request, mission_control::saveMission::Response &response)
{
    ROS_DEBUG("saveMissionCB");
    if(!MissionHandler::getInstance()->isLoaded())
        return false;

    if(!MissionHandler::getInstance()->save())
        return false;

    return true;
}

bool UiAPI::saveMissionAsCB(mission_control::saveMissionAs::Request &request, mission_control::saveMissionAs::Response &response)
{
    ROS_DEBUG_STREAM("saveMissionAsCB - name: " << request.name);
    if(!MissionHandler::getInstance()->isLoaded())
        return false;

    if(!MissionHandler::getInstance()->saveAs(request.name))
        return false;

    return true;
}

bool UiAPI::getMissionMetaCB(mission_control::getMissionMetaData::Request &request, mission_control::getMissionMetaData::Response &response)
{
    ROS_DEBUG_STREAM("getMissionMetaCB - name: " << request.name);
    if(!MissionHandler::getInstance()->isMission(request.name))
    {
        return false;
    }

    MissionParams params = MissionHandler::getInstance()->getMissionParams(request.name);

    response.cad = params.CAD_ref;
    response.name = params.name;
    response.description = params.description;
    response.last_saved = params.last_saved;
    response.number_of_tasks = params.number_of_tasks;
    response.state = params.state.toUInt();
    response.state_description = params.state.toString();

    return true;
}

bool UiAPI::createNewMissionCB(mission_control::createNewMission::Request &request, mission_control::createNewMission::Response &response)
{
    ROS_DEBUG_STREAM("createNewMissionCB - name: " << request.name);

    if(request.name == "")
    {
        response.success = false;
        response.description = "No name provided";
        return false;
    }

    if(MissionHandler::getInstance()->isLoaded())
    {
        MissionHandler::getInstance()->save();
    }

    if(!MissionHandler::getInstance()->createNew(request.name))
    {
        response.success = false;
        response.description = "Mission already exists";
        return false;
    }

    response.success = true;
    response.description = "succesfully created new mission";
    return true;
}

bool UiAPI::execStartCB(mission_control::execStart::Request &request, mission_control::execStart::Response &response)
{
    ROS_DEBUG_STREAM("execStartCB - name: " << request.name);

    if(request.name != "" && request.name != MissionHandler::getInstance()->getLoadedName())
    {
        //load the mission:
        if(!MissionHandler::getInstance()->load(request.name));
        {
            response.success = false;
            response.description = "Mission requested does not exist.";
        }
    }

    response.success = SystemEngine::getInstance()->execute();

    return true;
}

bool UiAPI::execAbortCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("execAbortCB");

    response.success = ExecutionEngine::getInstance()->abort();

    return true;
}

bool UiAPI::execPauseCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("execPauseCB");

    response.success = ExecutionEngine::getInstance()->pause();

    return true;
}

bool UiAPI::execSkipStudCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("execSkipStudCB");

    response.success = ExecutionEngine::getInstance()->skipStud();

    return true;
}

bool UiAPI::execSkipTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("execSkipTaskCB");

    response.success = ExecutionEngine::getInstance()->skipTask();

    return true;
}

bool UiAPI::execRetryCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("execRetryCB");

    response.success = ExecutionEngine::getInstance()->retry();

    return true;
}

bool UiAPI::instrStartCB(mission_control::execStart::Request &request, mission_control::execStart::Response &response)
{
    ROS_DEBUG_STREAM("instrStartCB - name: " << request.name);

    if(request.name != "" && request.name != MissionHandler::getInstance()->getLoadedName())
    {
        //load the mission:
        if(!MissionHandler::getInstance()->load(request.name));
        {
            response.success = false;
            response.description = "Mission requested does not exist.";
        }
    }

    response.success = SystemEngine::getInstance()->instruct();

    return true;
}

bool UiAPI::instrAbortCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("instrAbortCB");

    response.success = InstructionEngine::getInstance()->abort();

    return true;
}

bool UiAPI::instrPauseCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("instrPauseCB");

    response.success = InstructionEngine::getInstance()->pause();

    return true;
}

bool UiAPI::instrSkipTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("instrSkipTaskCB");

    response.success = InstructionEngine::getInstance()->skipTask();

    return true;
}

bool UiAPI::instrRetryCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("instrRetryCB");

    response.success = InstructionEngine::getInstance()->retry();

    return true;
}


void UiAPI::statePubTimeout(const ros::TimerEvent& event)
{
    mission_control::HardwareStates states;

    //robot state
    mission_control::DeviceState robot_state;
    robot_state.device_name = "robot";
    robot_state.device_state = HardwareStateMachine::getInstance()->getRobotState().toString();
    states.hardware_states.push_back(robot_state);

    //platform state
    mission_control::DeviceState platform_state;
    platform_state.device_name = "platform";
    platform_state.device_state = HardwareStateMachine::getInstance()->getPlatformState().toString();
    states.hardware_states.push_back(platform_state);

    //manipulator state
    mission_control::DeviceState manipulator_state;
    manipulator_state.device_name = "manipulator";
    manipulator_state.device_state = HardwareStateMachine::getInstance()->getManipulatorState().toString();
    states.hardware_states.push_back(manipulator_state);

    //system state
    mission_control::DeviceState system_state;
    manipulator_state.device_name = "system";
    manipulator_state.device_state = SystemEngine::getInstance()->current_state_.toString();
    states.hardware_states.push_back(manipulator_state);

    state_pub_.publish(states);
}
