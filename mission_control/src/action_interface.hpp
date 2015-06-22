#ifndef ACTION_INTERFACE_HPP_
#define ACTION_INTERFACE_HPP_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "actionlib/client/simple_action_client.h"
#include "mission_ctrl_msgs/executeWeldAction.h"
#include "mission_ctrl_msgs/movePlatformAction.h"
#include "execution_engine.hpp"

typedef actionlib::SimpleActionClient<mission_ctrl_msgs::movePlatformAction> movePlatform_client;
typedef actionlib::SimpleActionClient<mission_ctrl_msgs::executeWeldAction> executeWeld_client;

class ActionInterface
{
public:
    ActionInterface(ExecutionEngine* ee);
    ~ActionInterface();

    void sendArmGoal(mission_ctrl_msgs::executeWeldGoal goal);
    void sendPlatformGoal(mission_ctrl_msgs::movePlatformGoal goal);

    void cancelArmGoal();
    void cancelPlatformGoal();
    void cancelAllGoals();

private:

    void platformFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::movePlatformResultConstPtr &result);
    void platformActiveCB();
    void platformFeedbackCB(const mission_ctrl_msgs::movePlatformFeedbackConstPtr &feedback);

    void armFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::executeWeldResultConstPtr &result);
    void armActiveCB();
    void armFeedbackCB(const mission_ctrl_msgs::executeWeldFeedbackConstPtr &feedback);



    //action clients:
    ExecutionEngine* ee_;
    movePlatform_client* platform_client_;
    executeWeld_client* arm_client_;

};

#endif // ACTION_INTERFACE_HPP_
