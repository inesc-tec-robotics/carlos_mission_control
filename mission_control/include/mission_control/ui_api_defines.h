
#ifndef UIAPIDEFINES_H_
#define UIAPIDEFINES_H_

/* ROS Service namespaces */
#define UIAPI_CREATE_NEW_MISSION    "mission_control/UI/createNewMission"
#define UIAPI_LOAD_MISSION          "mission_control/UI/loadMission"
#define UIAPI_SAVE_MISSION          "mission_control/UI/saveMission"
#define UIAPI_SAVE_MISSION_AS       "mission_control/UI/saveMissionAs"
#define UIAPI_GET_MISSION_DATA      "mission_control/UI/getMissionMetaData"
#define UIAPI_GET_TASK_LIST         "mission_control/UI/getTaskList"
#define UIAPI_GET_MISSION_LIST      "mission_control/UI/getMissionList"
#define UIAPI_GET_MISSION_NAME      "mission_control/UI/getMissionName"
#define UIAPI_GET_TASK_DATA         "mission_control/UI/getTaskData"
#define UIAPI_GET_TASK_PARAMS       "mission_control/UI/getTaskParams"

#define UIAPI_EXEC_START            "mission_control/UI/execStart"
#define UIAPI_EXEC_ABORT            "mission_control/UI/execAbort"
#define UIAPI_EXEC_PAUSE_RESUME     "mission_control/UI/execPauseResume"
#define UIAPI_EXEC_RETRY            "mission_control/UI/execRetry"
#define UIAPI_EXEC_SKIP_TASK        "mission_control/UI/execSkipTask"
#define UIAPI_EXEC_SKIP_STUD        "mission_control/UI/execSkipStud"

#define UIAPI_INSTR_START           "mission_control/UI/instrStart"
#define UIAPI_INSTR_ABORT           "mission_control/UI/instrAbort"
#define UIAPI_INSTR_PAUSE_RESUME    "mission_control/UI/instrPauseResume"
#define UIAPI_INSTR_RETRY           "mission_control/UI/instrRetry"
#define UIAPI_INSTR_SKIP_TASK       "mission_control/UI/instrSkipTask"

#define UIAPI_EDIT_START            "mission_control/UI/editStart"
#define UIAPI_EDIT_STOP             "mission_control/UI/editStop"
#define UIAPI_SET_TASK_DATA         "mission_control/UI/setTaskData"
#define UIAPI_SET_MISSION_DATA      "mission_control/UI/setMissionData"
#define UIAPI_ADD_TASK              "mission_control/UI/addTask"
#define UIAPI_DELETE_TASK           "mission_control/UI/deleteTask"
#define UIAPI_GEN_TASKS             "mission_control/UI/generateTasks"


/* ROS Publisher namespaces */
#define UIAPI_EXEC_PROGRESS         "mission_control/UI/ExecProgress"
#define UIAPI_INSTR_PROGRESS        "mission_control/UI/InstrProgress"

#define UIAPI_HW_STATES             "mission_control/UI/HardwareStates"
#define UIAPI_HEART_BEAT            "mission_control/UI/HeartBeat"

#endif /* UIAPIDEFINES_H_ */
