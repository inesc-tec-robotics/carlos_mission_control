#ifndef ACTION_INTERFACE_HPP_
#define ACTION_INTERFACE_HPP_

class ExecutionEngine;
class InstructionEngine;

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "actionlib/client/simple_action_client.h"
#include "mission_ctrl_msgs/executeWeldAction.h"
#include "mission_ctrl_msgs/movePlatformAction.h"
#include "mission_ctrl_msgs/performTeachingAction.h"
#include "mission_ctrl_msgs/generateStudDistributionAction.h"

typedef actionlib::SimpleActionClient<mission_ctrl_msgs::movePlatformAction> movePlatform_client;
typedef actionlib::SimpleActionClient<mission_ctrl_msgs::executeWeldAction> executeWeld_client;
typedef actionlib::SimpleActionClient<mission_ctrl_msgs::performTeachingAction> performTeach_client;
typedef actionlib::SimpleActionClient<mission_ctrl_msgs::generateStudDistributionAction> generateStudPos_client;

class ActionInterface
{
public:
    ActionInterface();
    ~ActionInterface();

    void initExecution(ExecutionEngine* ee);
    void initInstruction(InstructionEngine* ie);

    void sendWeldGoal(std::string task_name);
    void sendMoveGoal(mission_ctrl_msgs::movePlatformGoal goal);

    void sendTeachGoal(std::string task_name);
    void sendGenPosGoal(std::string task_name);

    void cancelArmGoal();
    void cancelPlatformGoal();
    void cancelTeachGoal();
    void cancelGenPosGoal();
    void cancelExecGoals();
    void cancelInstrGoals();

    void removeArmGoal();
    void removePlatformGoal();
    void removeTeachGoal();
    void removeGenPosGoal();
    void removeInstrGoals();
    void removeExecGoals();

private:

    void platformFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::movePlatformResultConstPtr &result);
    void platformActiveCB();
    void platformFeedbackCB(const mission_ctrl_msgs::movePlatformFeedbackConstPtr &feedback);

    void armFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::executeWeldResultConstPtr &result);
    void armActiveCB();
    void armFeedbackCB(const mission_ctrl_msgs::executeWeldFeedbackConstPtr &feedback);

    void teachFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::performTeachingResultConstPtr &result);
    void teachActiveCB();
    void teachFeedbackCB(const mission_ctrl_msgs::performTeachingFeedbackConstPtr &feedback);

    void genPosFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::generateStudDistributionResultConstPtr &result);
    void genPosActiveCB();
    void genPosFeedbackCB(const mission_ctrl_msgs::generateStudDistributionFeedbackConstPtr &feedback);

    //pointer to parents:
    ExecutionEngine* ee_;
    InstructionEngine* ie_;

    //action clients:
    movePlatform_client* move_client_;
    executeWeld_client* weld_client_;
    performTeach_client* teach_client_;
    generateStudPos_client* gen_pos_client_;
};

#endif // ACTION_INTERFACE_HPP_
