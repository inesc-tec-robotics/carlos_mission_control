/* Created by Casper Schou @ AAU 2015
 *
 * The mission handler does not itself incorporate an integrated UI.
 * Instead, this class provides a set of ROS services and topics for
 * interaction with the mission controller. Thus, this class provides an
 * API for UIs to utilize the mission controller.
 *
 * This class implements a singelton pattern to make access to the same object from various classes easier
 */

#ifndef UIAPI_HPP_
#define UIAPI_HPP_

//=================================
// Included dependencies
//=================================
#include <string>
#include <vector>
#include "ros/service_server.h"
#include "ros/publisher.h"
#include "ros/timer.h"
#include "mission_control/getTaskList.h"
#include "mission_control/getMissionList.h"
#include "mission_control/getMissionData.h"
#include "mission_control/getTaskData.h"
#include "mission_control/Progress.h"
#include "mission_control/Trigger.h"
#include "mission_control/setMissionData.h"
#include "mission_control/setTaskData.h"
#include "mission_control/getTaskParams.h"

class UiAPI
{

public:
    static UiAPI* getInstance();

    ~UiAPI();

    /* Trigger function for execution progress update
     * This funciton is called from the ExecutionEngine
     * and signals that the progress information has changed.
     * The function will then publish the new information to the
     * ros topic.
     */
    void execProgressUpdate(mission_control::Progress &progress);

    /* Trigger function for instruction progress update
     * This funciton is called from the InstructionEngine
     * and signals that the progress information has changed.
     * The function will then publish the new information to the
     * ros topic.
     */
    void instrProgressUpdate(mission_control::Progress &progress);

private:

    UiAPI();

    //Callback function for ROS Service Servers:
    bool getMissionListCB(mission_control::getMissionList::Request &request, mission_control::getMissionList::Response &response);
    bool getTaskListCB(mission_control::getTaskList::Request &request, mission_control::getTaskList::Response &response);
    bool getMissionDataCB(mission_control::getMissionData::Request &request, mission_control::getMissionData::Response &response);
    bool getMissionNameCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);    //message is used to return the name
    bool getTaskDataCB(mission_control::getTaskData::Request &request, mission_control::getTaskData::Response &response);
    bool getTaskParamsCB(mission_control::getTaskParams::Request &request, mission_control::getTaskParams::Response &response); //get params for a task - including default value, range etc.

    bool loadMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool saveMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool saveMissionAsCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool createNewMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);

    bool execStartCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execAbortCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execKillCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execPauseCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execSkipStudCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execSkipTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execRetryCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);

    bool instrStartCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrAbortCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrKillCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrPauseCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrSkipStudCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrSkipTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrRetryCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);

    bool editStartCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool editStopCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);

    bool setMissionDataCB(mission_control::setMissionData::Request &request, mission_control::setMissionData::Response &response);
    bool setTaskDataCB(mission_control::setTaskData::Request &request, mission_control::setTaskData::Response &response);
    bool addTaskCB(mission_control::setTaskData::Request &request, mission_control::setTaskData::Response &response);
    bool deleteTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool genTasksCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);

    /* Callback function for state publisher timeout event.
     * This function will publish the state of each hardware
     * sub-system and the overall system state to a ros topic.
     * The publication frequency is controller by the
     * state_pub_timer_.
     */
    void statePubTimeout(const ros::TimerEvent &event);

    /* Callback function for hear beat timeout event.
     * The UiAPI publishes a heart beat (boolean true) at
     * a given frequency. This is used to signal any connected
     * GUI that the system is running.
     */
    void heartBeatPubTimeout(const ros::TimerEvent &event);

    static UiAPI* instance_;

    ros::NodeHandle n;

    //mission management services
    ros::ServiceServer create_new_mission_srv_;
    ros::ServiceServer load_mission_srv_;
    ros::ServiceServer save_mission_srv_;
    ros::ServiceServer save_mission_as_srv_;

    //data retreival services
    ros::ServiceServer get_mission_data_srv_;
    ros::ServiceServer get_task_list_srv_;
    ros::ServiceServer get_mission_list_srv_;
    ros::ServiceServer get_mission_name_srv_;
    ros::ServiceServer get_task_data_srv_;
    ros::ServiceServer get_task_params_srv_;

    //data change services
    ros::ServiceServer set_mission_data_srv_;
    ros::ServiceServer set_task_data_srv_;
    ros::ServiceServer add_task_data_srv_;
    ros::ServiceServer gen_tasks_srv_;
    ros::ServiceServer delete_task_srv_;

    //execution control services
    ros::ServiceServer exec_start_srv_;
    ros::ServiceServer exec_abort_srv_;
    ros::ServiceServer exec_pause_srv_;
    ros::ServiceServer exec_skip_stud_srv_;
    ros::ServiceServer exec_skip_task_srv_;
    ros::ServiceServer exec_retry_srv_;
    ros::ServiceServer exec_kill_srv_;

    //instruction control services
    ros::ServiceServer instr_start_srv_;
    ros::ServiceServer instr_abort_srv_;
    ros::ServiceServer instr_pause_srv_;
    ros::ServiceServer instr_skip_task_srv_;
    ros::ServiceServer instr_retry_srv_;
    ros::ServiceServer instr_kill_srv_;

    //edit control services
    ros::ServiceServer edit_start_srv_;
    ros::ServiceServer edit_stop_srv_;

    //Publishers
    ros::Publisher state_pub_;
    ros::Publisher heart_beat_pub_;
    ros::Publisher ui_message_pub_;
    ros::Publisher exec_progress_pub_;
    ros::Publisher instr_progress_pub_;

    //Timers
    ros::Timer state_pub_timer_;
    ros::Timer heart_beat_pub_timer_;
};

#endif /* UIAPI_HPP_ */
