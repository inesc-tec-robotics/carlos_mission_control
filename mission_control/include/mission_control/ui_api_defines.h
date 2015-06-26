
#ifndef UIAPIDEFINES_H_
#define UIAPIDEFINES_H_

/* ROS Service namespaces */
#define UIAPI_CREATE_NEW_MISSION    "mission_control/UI/createNewMission"
#define UIAPI_LOAD_MISSION          "mission_control/UI/loadMission"
#define UIAPI_SAVE_MISSION          "mission_control/UI/saveMission"
#define UIAPI_SAVE_MISSION_AS       "mission_control/UI/saveMissionAs"
#define UIAPI_GET_MISSION_META      "mission_control/UI/getMissionMetaData"
#define UIAPI_GET_TASK_LIST         "mission_control/UI/getTaskList"
#define UIAPI_GET_MISSION_LIST      "mission_control/UI/getMissionList"

#define UIAPI_EXEC_START            "mission_control/UI/execStart"
#define UIAPI_EXEC_ABORT            "mission_control/UI/execAbort"
#define UIAPI_EXEC_PAUSE_RESUME     "mission_control/UI/execPauseResume"
#define UIAPI_EXEC_RETRY            "mission_control/UI/execRetry"
#define UIAPI_EXEC_SKIP_TASK        "mission_control/UI/execSkipTask"
#define UIAPI_EXEC_SKIP_STUD        "mission_control/UI/execSkipStud"

#define UIAPI_INSTR_START            "mission_control/UI/instrStart"
#define UIAPI_INSTR_ABORT            "mission_control/UI/instrAbort"
#define UIAPI_INSTR_PAUSE_RESUME     "mission_control/UI/instrPauseResume"
#define UIAPI_INSTR_RETRY            "mission_control/UI/instrRetry"
#define UIAPI_INSTR_SKIP_TASK        "mission_control/UI/instrSkipTask"

/* ROS Publisher namespaces */
#define UIAPI_EXEC_PROGRESS         "mission_control/UI/ExecProgress"
#define UIAPI_INSTR_PROGRESS        "mission_control/UI/InstrProgress"

#define UIAPI_HW_STATES             "mission_control/UI/HardwareStates"

#endif /* UIAPIDEFINES_H_ */
