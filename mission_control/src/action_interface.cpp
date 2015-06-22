
#include "action_interface.hpp"
#include "mission_handler.hpp"
#include "mission_ctrl_msgs/mission_ctrl_defines.h"

#define WAIT_FOR_SERVER_TIMEOUT 3.0 // sec

using namespace std;

ActionInterface::ActionInterface(ExecutionEngine* ee) : ee_(ee)
{
    //initialise action clients
    platform_client_ = new actionlib::SimpleActionClient<mission_ctrl_msgs::movePlatformAction>(CARLOS_MOVE_ACTION, true);
    arm_client_ = new actionlib::SimpleActionClient<mission_ctrl_msgs::executeWeldAction>(CARLOS_WELD_ACTION, true);

    platform_client_->waitForServer(ros::Duration(WAIT_FOR_SERVER_TIMEOUT));
    arm_client_->waitForServer(ros::Duration(WAIT_FOR_SERVER_TIMEOUT));
}

ActionInterface::~ActionInterface()
{
    delete platform_client_;
    delete arm_client_;
}

void ActionInterface::sendArmGoal(mission_ctrl_msgs::executeWeldGoal goal)
{
    arm_client_->sendGoal(
            goal,
            boost::bind(&ActionInterface::armFinishedCB, this, _1, _2),
            boost::bind(&ActionInterface::armActiveCB, this),
            boost::bind(&ActionInterface::armFeedbackCB, this, _1));
}

void ActionInterface::sendPlatformGoal(mission_ctrl_msgs::movePlatformGoal goal)
{
    cout << "sending goal..." << endl;

    platform_client_->sendGoal(
            goal,
            boost::bind(&ActionInterface::platformFinishedCB, this, _1, _2),
            boost::bind(&ActionInterface::platformActiveCB, this),
            boost::bind(&ActionInterface::platformFeedbackCB, this, _1));

    cout << "goal send!" << endl;
}

void ActionInterface::cancelArmGoal()
{
    arm_client_->cancelAllGoals();
}

void ActionInterface::cancelPlatformGoal()
{
    platform_client_->cancelAllGoals();
}

void ActionInterface::cancelAllGoals()
{
    cancelPlatformGoal();
    cancelArmGoal();
}

void ActionInterface::platformFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::movePlatformResultConstPtr &result)
{
    switch(state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ee_->navDone();
        break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
        ee_->goalCancelled();
        break;
    case actionlib::SimpleClientGoalState::ABORTED:
        ee_->navFailed();
        break;
    case actionlib::SimpleClientGoalState::LOST:
        ee_->navFailed();
        break;
    default:
        ee_->navFailed();
        break;
    }

}

void ActionInterface::platformActiveCB()
{

}

void ActionInterface::platformFeedbackCB(const mission_ctrl_msgs::movePlatformFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Received feedback from platform - should not happen?");
}

void ActionInterface::armFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::executeWeldResultConstPtr &result)
{
    //msm::back::state_machine<ExecutionStateMachine_> &esm = static_cast<msm::back::state_machine<ExecutionStateMachine_> &>(*this);

    switch(state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ee_->maniDone();
        break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
        break;
    case actionlib::SimpleClientGoalState::ABORTED:
        ee_->maniFailed();
        break;
    case actionlib::SimpleClientGoalState::LOST:
        ee_->maniFailed();
        break;
    default:
        ee_->maniFailed();
        break;
    }

}

void ActionInterface::armActiveCB()
{

}

void ActionInterface::armFeedbackCB(const mission_ctrl_msgs::executeWeldFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Received feedback from arm");
    MissionHandler::getInstance()->setStudState(ee_->getCurrentTask(),feedback->stud_id,(feedback->stud_state ? stud::SUCCEEDED : stud::FAILED)); //ain't complex code just a beauty?!
    ee_->maniFeedback();
}
