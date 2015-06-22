#include "ros/node_handle.h"
#include "mission_handler.hpp"
#include "UI_API.hpp"
#include "hw_state_machine.hpp"
#include "mission_control/HardwareStates.h"
#include "system_engine.hpp"
#include "execution_engine.hpp"

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
    get_task_list_srv_ = n.advertiseService("mission_control/UI/getTaskList", &UiAPI::getTaskListCB, this);
    get_mission_list_srv_ = n.advertiseService("mission_control/UI/getMissionList", &UiAPI::getMissionListCB, this);
    load_mission_srv_ = n.advertiseService("mission_control/UI/loadMission", &UiAPI::loadMissionCB, this);
    save_mission_srv_ = n.advertiseService("mission_control/UI/saveMission", &UiAPI::saveMissionCB, this);
    save_mission_as_srv_ = n.advertiseService("mission_control/UI/saveMissionAs", &UiAPI::saveMissionAsCB, this);
    get_mission_meta_srv_ = n.advertiseService("mission_control/UI/getMissionMetaData", &UiAPI::getMissionMetaCB, this);
    exec_start_srv_ = n.advertiseService("mission_control/UI/execStart", &UiAPI::execStartCB, this);
    exec_abort_srv_ = n.advertiseService("mission_control/UI/execAbort", &UiAPI::execAbortCB, this);
    exec_pause_srv_ = n.advertiseService("mission_control/UI/execPauseResume", &UiAPI::execPauseCB, this);
    create_new_mission_srv_ = n.advertiseService("mission_control/UI/createNewMission", &UiAPI::createNewMissionCB, this);
    exec_skip_stud_srv_ = n.advertiseService("mission_control/UI/execSkipStud", &UiAPI::execSkipStudCB, this);
    exec_skip_task_srv_ = n.advertiseService("mission_control/UI/execSkipTask", &UiAPI::execSkipTaskCB, this);
    exec_retry_srv_ = n.advertiseService("mission_control/UI/execRetry", &UiAPI::execRetryCB, this);

    ROS_INFO("ROS ServiceServers for UI API started");

    //publisher for execution progress:
    exec_progress_pub_ = n.advertise<mission_control::ExecProgress>("mission_control/UI/ExecProgress",10);

    //create publisher of states:
    state_pub_ = n.advertise<mission_control::HardwareStates>("mission_control/UI/HardwareStates",10);

    //create timer for state publishing:
    state_pub_timer_ = n.createTimer(ros::Duration((1/PUB_FREQ)), &UiAPI::statePubTimeout, this);
}

UiAPI::~UiAPI()
{
}

void UiAPI::execProgressUpdate(mission_control::ExecProgress &progress)
{
    exec_progress_pub_.publish(progress);
}

bool UiAPI::getTaskListCB(mission_control::getTaskList::Request &request, mission_control::getTaskList::Response &response)
{
    ROS_INFO("getTaskListCB");
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
    ROS_INFO("getMissionListCB");
    vector<string> missions = MissionHandler::getInstance()->getListOfMissions();

    response.missions = missions;

    return true;
}

bool UiAPI::loadMissionCB(mission_control::loadMission::Request &request, mission_control::loadMission::Response &response)
{
    ROS_INFO_STREAM("loadMissionCB - name: " << request.name);
    if(!MissionHandler::getInstance()->isMission(request.name))
        return false;

    if(!MissionHandler::getInstance()->load(request.name))
        return false;

    return true;
}

bool UiAPI::saveMissionCB(mission_control::saveMission::Request &request, mission_control::saveMission::Response &response)
{
    ROS_INFO("saveMissionCB");
    if(!MissionHandler::getInstance()->isLoaded())
        return false;

    if(!MissionHandler::getInstance()->save())
        return false;

    return true;
}

bool UiAPI::saveMissionAsCB(mission_control::saveMissionAs::Request &request, mission_control::saveMissionAs::Response &response)
{
    ROS_INFO_STREAM("saveMissionAsCB - name: " << request.name);
    if(!MissionHandler::getInstance()->isLoaded())
        return false;

    if(!MissionHandler::getInstance()->saveAs(request.name))
        return false;

    return true;
}

bool UiAPI::getMissionMetaCB(mission_control::getMissionMetaData::Request &request, mission_control::getMissionMetaData::Response &response)
{
    ROS_INFO_STREAM("getMissionMetaCB - name: " << request.name);
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
    ROS_INFO_STREAM("createNewMissionCB - name: " << request.name);

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
    ROS_INFO_STREAM("execStartCB - name: " << request.name);

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

bool UiAPI::execAbortCB(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    ROS_INFO_STREAM("execAbortCB");

    response.success = ExecutionEngine::getInstance()->abort();

    return true;
}

bool UiAPI::execPauseCB(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    ROS_INFO_STREAM("execPauseCB");

    response.success = ExecutionEngine::getInstance()->pause();

    return true;
}

bool UiAPI::execSkipStudCB(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    ROS_INFO_STREAM("execSkipStudCB");

    response.success = ExecutionEngine::getInstance()->skipStud();

    return true;
}

bool UiAPI::execSkipTaskCB(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    ROS_INFO_STREAM("execSkipTaskCB");

    response.success = ExecutionEngine::getInstance()->skipTask();

    return true;
}

bool UiAPI::execRetryCB(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    ROS_INFO_STREAM("execRetryCB");

    response.success = ExecutionEngine::getInstance()->retry();

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

    state_pub_.publish(states);
}
