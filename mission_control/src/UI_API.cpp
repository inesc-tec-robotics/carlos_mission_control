#include "ros/node_handle.h"
#include "mission_handler.hpp"
#include "UI_API.hpp"
#include "hw_state_machine.hpp"
#include "std_msgs/Bool.h"
#include "mission_control/HardwareStates.h"
#include "system_engine.hpp"
#include "execution_engine.hpp"
#include "instruction_engine.hpp"
#include "mission_control/ui_api_defines.h"

#define STATE_PUB_FREQ 10.0 //publication frequency in Hz for hardware states
#define HB_PUB_FREQ 1.0 //publication frequency in Hz for hear beat

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
    get_mission_data_srv_ = n.advertiseService(UIAPI_GET_MISSION_DATA, &UiAPI::getMissionDataCB, this);
    get_task_list_srv_ = n.advertiseService(UIAPI_GET_TASK_LIST, &UiAPI::getTaskListCB, this);
    get_mission_list_srv_ = n.advertiseService(UIAPI_GET_MISSION_LIST, &UiAPI::getMissionListCB, this);
    get_mission_name_srv_ = n.advertiseService(UIAPI_GET_MISSION_NAME, &UiAPI::getMissionNameCB, this);
    get_task_data_srv_ = n.advertiseService(UIAPI_GET_TASK_DATA, &UiAPI::getTaskDataCB, this);
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
    edit_start_srv_ = n.advertiseService(UIAPI_EDIT_START, &UiAPI::editStartCB, this);
    edit_stop_srv_ = n.advertiseService(UIAPI_EDIT_STOP, &UiAPI::editStopCB, this);
    set_mission_data_srv_ = n.advertiseService(UIAPI_SET_MISSION_DATA, &UiAPI::setMissionDataCB, this);
    set_task_data_srv_ = n.advertiseService(UIAPI_SET_TASK_DATA, &UiAPI::setTaskDataCB, this);
    add_task_data_srv_ = n.advertiseService(UIAPI_ADD_TASK, &UiAPI::addTaskCB, this);
    delete_task_srv_ = n.advertiseService(UIAPI_DELETE_TASK, &UiAPI::deleteTaskCB, this);


    ROS_INFO("ROS ServiceServers for UI API started");

    //publisher for execution progress:
    exec_progress_pub_ = n.advertise<mission_control::Progress>(UIAPI_EXEC_PROGRESS,10, true);

    //publisher for instruction progress:
    instr_progress_pub_ = n.advertise<mission_control::Progress>(UIAPI_INSTR_PROGRESS,10, true);

    //create publisher of states:
    state_pub_ = n.advertise<mission_control::HardwareStates>(UIAPI_HW_STATES,10);

    //create publisher of heart beat:
    heart_beat_pub_ = n.advertise<std_msgs::Bool>(UIAPI_HEART_BEAT,10);

    //create timer for state publishing:
    state_pub_timer_ = n.createTimer(ros::Duration((1/STATE_PUB_FREQ)), &UiAPI::statePubTimeout, this);

    //create timer for state publishing:
    heart_beat_pub_timer_ = n.createTimer(ros::Duration((1/HB_PUB_FREQ)), &UiAPI::heartBeatPubTimeout, this);
}

UiAPI::~UiAPI()
{
    //notisfy UI that mission control is (intentionally) shutting down.
    cout << "UIAPI shutting down" << endl;
    std_msgs::Bool msg;
    msg.data = false;
    heart_beat_pub_.publish(msg);
    //usleep(5000000);
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

bool UiAPI::loadMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("loadMissionCB - name: " << request.input);

    //check with system engine, if the mission is locked!
    if(SystemEngine::getInstance()->isMissionLocked())
    {
        response.success = false;
        response.message = "Mission currently locked";
        return true;
    }

    if(!MissionHandler::getInstance()->isMission(request.input))
    {
        response.success = false;
        response.message = request.input + " is not a valid mission";
        return true;
    }

    if(!MissionHandler::getInstance()->load(request.input))
    {
        response.success = false;
        response.message = "Load failed";
        return true;
    }

    response.success = true;
    return true;
}

bool UiAPI::saveMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG("saveMissionCB");
    response.success = false;
    if(!MissionHandler::getInstance()->isLoaded())
        return false;

    if(!MissionHandler::getInstance()->save())
        return false;

    response.success = true;

    return true;
}

bool UiAPI::saveMissionAsCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("saveMissionAsCB - name: " << request.input);

    //check with system engine, if the mission is locked!
    if(SystemEngine::getInstance()->isMissionLocked())
    {
        response.success = false;
        response.message = "Mission currently locked";
        return true;
    }

    if(!MissionHandler::getInstance()->isLoaded())
    {
        response.success = false;
        response.message = "No name given for save as";
        return true;
    }

    if(!MissionHandler::getInstance()->saveAs(request.input))
    {
        response.success = false;
        response.message = "Save as failed";
        return true;
    }

    return true;
}

bool UiAPI::getMissionDataCB(mission_control::getMissionData::Request &request, mission_control::getMissionData::Response &response)
{
    ROS_DEBUG_STREAM("getMissionMetaCB - name: " << request.name);

    MissionData data;

    if(request.name == "")  //hence request for currently loaded:
    {
        if(!MissionHandler::getInstance()->isLoaded())
        {
            response.success = false;
            response.message = "No mission loaded";
            return true;
        }
        data = MissionHandler::getInstance()->getMissionData();
    }
    else
    {
        if(!MissionHandler::getInstance()->isMission(request.name))
        {
            response.success = false;
            response.message = request.name + " is not a valid mission";
            return true;
        }
        data = MissionHandler::getInstance()->getMissionData(request.name);
    }

    response.data = data.toMsg();
    response.success = true;

    return true;
}

bool UiAPI::getMissionNameCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("getMissionNameCB");

    response.message = MissionHandler::getInstance()->getLoadedName();
    response.success = true;

    return true;
}

bool UiAPI::getTaskDataCB(mission_control::getTaskData::Request &request, mission_control::getTaskData::Response &response)
{

    TaskData data = MissionHandler::getInstance()->getTaskData(request.name);

    if(request.name != data.name)
    {
        response.success = false;
        response.message = "Inconsistent name of task. Should not happen!";
        return true;
    }

    response.data = data.toMsg();
    response.success = true;

    return true;
}

bool UiAPI::createNewMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("createNewMissionCB - name: " << request.input);

    //check with system engine, if the mission is locked!
    if(SystemEngine::getInstance()->isMissionLocked())
    {
        response.success = false;
        response.message = "Mission currently locked";
        return true;
    }

    if(request.input == "")
    {
        response.success = false;
        response.message = "No name provided";
        return false;
    }

    if(MissionHandler::getInstance()->isLoaded())
    {
        MissionHandler::getInstance()->save();
    }

    if(!MissionHandler::getInstance()->createNew(request.input))
    {
        response.success = false;
        response.message = "Mission already exists";
        return false;
    }

    response.success = true;
    response.message = "succesfully created new mission";
    return true;
}

bool UiAPI::execStartCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("execStartCB - name: " << request.input);

    if(request.input != "" && request.input != MissionHandler::getInstance()->getLoadedName())
    {
        //load the mission:
        if(!MissionHandler::getInstance()->load(request.input));
        {
            response.success = false;
            response.message = "Mission requested does not exist.";
            return true;
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

bool UiAPI::instrStartCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("instrStartCB - name: " << request.input);

    if(request.input != "" && request.input != MissionHandler::getInstance()->getLoadedName())
    {
        //load the mission:
        if(!MissionHandler::getInstance()->load(request.input));
        {
            response.success = false;
            response.message = "Mission requested does not exist.";
            return true;
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

bool UiAPI::editStartCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_INFO_STREAM("editStartCB - name: " << request.input);

    if(request.input != "" && request.input != MissionHandler::getInstance()->getLoadedName())
    {
        //load the mission:
        if(!MissionHandler::getInstance()->load(request.input));
        {
            response.success = false;
            response.message = "Mission requested does not exist.";
            return true;
        }
    }

    response.success = SystemEngine::getInstance()->edit();

    return true;
}

bool UiAPI::editStopCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("editStopCB");

    response.success = SystemEngine::getInstance()->editDone();

    return true;
}

bool UiAPI::setMissionDataCB(mission_control::setMissionData::Request &request, mission_control::setMissionData::Response &response)
{
    ROS_DEBUG_STREAM("setMissionDataCB");

    //check with system engine, if edit is allowed!
    if(!SystemEngine::getInstance()->isEditAllowed())
    {
        response.success = false;
        response.message = "Not in edit mode";
        return true;
    }

    mission_control::MissionData data;
    data.cad = request.cad;
    data.description = request.description;
    data.name = request.name;

    response.success = MissionHandler::getInstance()->setMissionData(data);

    return true;
}

bool UiAPI::setTaskDataCB(mission_control::setTaskData::Request &request, mission_control::setTaskData::Response &response)
{
    ROS_DEBUG_STREAM("setTaskDataCB");

    //check with system engine, if edit is allowed!
    if(!SystemEngine::getInstance()->isEditAllowed())
    {
        response.success = false;
        response.message = "Not in edit mode";
        return true;
    }

    response.success = MissionHandler::getInstance()->setTaskData(request.name, request.data);

    return true;
}

bool UiAPI::addTaskCB(mission_control::setTaskData::Request &request, mission_control::setTaskData::Response &response)
{
    ROS_DEBUG_STREAM("addTaskCB");

    //check with system engine, if edit is allowed!
    if(!SystemEngine::getInstance()->isEditAllowed())
    {
        response.success = false;
        response.message = "Not in edit mode";
        return true;
    }

    response.success = MissionHandler::getInstance()->addTask(request.data, request.name);

    return true;
}

bool UiAPI::deleteTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response)
{
    ROS_DEBUG_STREAM("deleteTaskCB");

    //check with system engine, if edit is allowed!
    if(!SystemEngine::getInstance()->isEditAllowed())
    {
        response.success = false;
        response.message = "Not in edit mode";
        return true;
    }

    response.success = MissionHandler::getInstance()->deleteTask(request.input);
    response.message = "Failed to delete task";

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

void UiAPI::heartBeatPubTimeout(const ros::TimerEvent &event)
{
    std_msgs::Bool msg;
    msg.data = true;
    heart_beat_pub_.publish(msg);
}
