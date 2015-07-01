
#ifndef UIAPI_HPP_
#define UIAPI_HPP_

//=================================
// Forward declared dependencies
//=================================
//class ExecutionEngine;
//class SystemEngine;
//class MissionHandler;

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
#include "mission_control/loadMission.h"
#include "mission_control/saveMissionAs.h"
#include "mission_control/getMissionMetaData.h"
#include "mission_control/getTaskParams.h"
#include "mission_control/Start.h"
#include "mission_control/createNewMission.h"
#include "mission_control/Progress.h"
#include "mission_control/Trigger.h"



class UiAPI
{

public:
    static UiAPI* getInstance();
//    {
//        static UiAPI instance;
//        return instance;
//    }

    ~UiAPI();

    void execProgressUpdate(mission_control::Progress &progress);
    void instrProgressUpdate(mission_control::Progress &progress);

private:

    UiAPI();

    bool getMissionListCB(mission_control::getMissionList::Request &request, mission_control::getMissionList::Response &response);
    bool getTaskListCB(mission_control::getTaskList::Request &request, mission_control::getTaskList::Response &response);
    bool getMissionMetaCB(mission_control::getMissionMetaData::Request &request, mission_control::getMissionMetaData::Response &response);
    bool getMissionNameCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);    //message is used to return the name
    bool getTaskParamsCB(mission_control::getTaskParams::Request &request, mission_control::getTaskParams::Response &response);

    bool loadMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool saveMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool saveMissionAsCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool createNewMissionCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);

    bool execStartCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execAbortCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execPauseCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execSkipStudCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execSkipTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execRetryCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);

    bool instrStartCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrAbortCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrPauseCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrSkipStudCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrSkipTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool instrRetryCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);


    void statePubTimeout(const ros::TimerEvent &event);
    void heartBeatPubTimeout(const ros::TimerEvent &event);

    //variables
    static UiAPI* instance_;

    ros::NodeHandle n;

    //mission management services
    ros::ServiceServer create_new_mission_srv_;
    ros::ServiceServer load_mission_srv_;
    ros::ServiceServer save_mission_srv_;
    ros::ServiceServer save_mission_as_srv_;

    //data retreival services
    ros::ServiceServer get_mission_meta_srv_;
    ros::ServiceServer get_task_list_srv_;
    ros::ServiceServer get_mission_list_srv_;
    ros::ServiceServer get_mission_name_srv_;
    ros::ServiceServer get_task_params_srv_;

    //execution control services
    ros::ServiceServer exec_start_srv_;
    ros::ServiceServer exec_abort_srv_;
    ros::ServiceServer exec_pause_srv_;
    ros::ServiceServer exec_skip_stud_srv_;
    ros::ServiceServer exec_skip_task_srv_;
    ros::ServiceServer exec_retry_srv_;

    //instruction control services
    ros::ServiceServer instr_start_srv_;
    ros::ServiceServer instr_abort_srv_;
    ros::ServiceServer instr_pause_srv_;
    ros::ServiceServer instr_skip_task_srv_;
    ros::ServiceServer instr_retry_srv_;


    ros::Publisher state_pub_;
    ros::Publisher heart_beat_pub_;
    ros::Publisher ui_message_pub_;
    ros::Publisher exec_progress_pub_;
    ros::Publisher instr_progress_pub_;

    ros::Timer state_pub_timer_;
    ros::Timer heart_beat_pub_timer_;
};

#endif /* UIAPI_HPP_ */
