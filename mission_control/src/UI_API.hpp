
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
#include "mission_control/saveMission.h"
#include "mission_control/saveMissionAs.h"
#include "mission_control/getMissionMetaData.h"
#include "mission_control/execStart.h"
#include "mission_control/execAbort.h"
#include "mission_control/execPause.h"
#include "mission_control/createNewMission.h"
#include "mission_control/Progress.h"
#include "mission_control/Trigger.h"



class UiAPI
{

public:
    static UiAPI* getInstance();

    ~UiAPI();

    void execProgressUpdate(mission_control::Progress &progress);
    void instrProgressUpdate(mission_control::Progress &progress);

private:

    UiAPI();

    bool getMissionListCB(mission_control::getMissionList::Request &request, mission_control::getMissionList::Response &response);
    bool getTaskListCB(mission_control::getTaskList::Request &request, mission_control::getTaskList::Response &response);
    bool loadMissionCB(mission_control::loadMission::Request &request, mission_control::loadMission::Response &response);
    bool saveMissionCB(mission_control::saveMission::Request &request, mission_control::saveMission::Response &response);
    bool saveMissionAsCB(mission_control::saveMissionAs::Request &request, mission_control::saveMissionAs::Response &response);
    bool getMissionMetaCB(mission_control::getMissionMetaData::Request &request, mission_control::getMissionMetaData::Response &response);
    bool createNewMissionCB(mission_control::createNewMission::Request &request, mission_control::createNewMission::Response &response);

    bool execStartCB(mission_control::execStart::Request &request, mission_control::execStart::Response &response);
    bool execAbortCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execPauseCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execSkipStudCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execSkipTaskCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);
    bool execRetryCB(mission_control::Trigger::Request &request, mission_control::Trigger::Response &response);


    void statePubTimeout(const ros::TimerEvent &event);

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

    //execution control services
    ros::ServiceServer exec_start_srv_;
    ros::ServiceServer exec_abort_srv_;
    ros::ServiceServer exec_pause_srv_;
    ros::ServiceServer exec_skip_stud_srv_;
    ros::ServiceServer exec_skip_task_srv_;
    ros::ServiceServer exec_retry_srv_;


    ros::Publisher state_pub_;
    ros::Publisher ui_message_pub_;
    ros::Publisher exec_progress_pub_;
    ros::Publisher instr_progress_pub_;

    ros::Timer state_pub_timer_;
};

#endif /* UIAPI_HPP_ */
