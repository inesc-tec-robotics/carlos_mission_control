
#include "action_interface.hpp"
#include "mission_handler.hpp"
#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include "execution_engine.hpp"
#include "instruction_engine.hpp"

#define WAIT_FOR_SERVER_TIMEOUT 1.0 // sec

using namespace std;

ActionInterface::ActionInterface()
{
}

ActionInterface::~ActionInterface()
{
    if(move_client_ != NULL)
        delete move_client_;

    if(weld_client_ != NULL)
        delete weld_client_;

    if(teach_client_ != NULL)
        delete teach_client_;

    if(gen_pos_client_ != NULL)
        delete gen_pos_client_;
}

void ActionInterface::initExecution(ExecutionEngine* ee)
{
    ee_ = ee;
    move_client_ = new actionlib::SimpleActionClient<mission_ctrl_msgs::movePlatformAction>(CARLOS_MOVE_ACTION, true);
    weld_client_ = new actionlib::SimpleActionClient<mission_ctrl_msgs::executeWeldAction>(CARLOS_WELD_ACTION, true);
    if(!move_client_->waitForServer(ros::Duration(WAIT_FOR_SERVER_TIMEOUT)))
    {
        ROS_DEBUG_STREAM("Waiting for the move platform action server to become online timed out after " << WAIT_FOR_SERVER_TIMEOUT << " seconds. If not online by the time of usage, the execution engine or instruction engine will go into <hardware_error> state");
        ROS_WARN("<move platform> action server not online. Please make sure it is online before attempting utilize the platform");
    }
    if(!weld_client_->waitForServer(ros::Duration(WAIT_FOR_SERVER_TIMEOUT)))
    {
        ROS_DEBUG_STREAM("Waiting for the execute weld action server to become online timed out after " << WAIT_FOR_SERVER_TIMEOUT << " seconds. If not online by the time of usage, the execution engine or instruction engine will go into <hardware_error> state");
        ROS_WARN("<execute weld> action server not online. Please make sure it is online before attempting execute any tasks");
    }
}

void ActionInterface::initInstruction(InstructionEngine* ie)
{
    ie_ = ie;
    teach_client_ = new actionlib::SimpleActionClient<mission_ctrl_msgs::performTeachingAction>(CARLOS_TEACHING_ACTION, true);
    gen_pos_client_ = new actionlib::SimpleActionClient<mission_ctrl_msgs::generateStudDistributionAction>(CARLOS_DISTRIBUTION_ACTION, true);
    move_client_ = new actionlib::SimpleActionClient<mission_ctrl_msgs::movePlatformAction>(CARLOS_MOVE_ACTION, true);
    if(!teach_client_->waitForServer(ros::Duration(WAIT_FOR_SERVER_TIMEOUT)))
    {
        ROS_DEBUG_STREAM("Waiting for the teach action server to become online timed out after " << WAIT_FOR_SERVER_TIMEOUT << " seconds. If not online by the time of usage, the execution engine or instruction engine will go into <hardware_error> state");
        ROS_WARN("<teach> action server not online. Please make sure it is online before attempting to instruct any tasks");
    }
    if(!gen_pos_client_->waitForServer(ros::Duration(WAIT_FOR_SERVER_TIMEOUT)))
    {
        ROS_DEBUG_STREAM("Waiting for the generate stud position action server to become online timed out after " << WAIT_FOR_SERVER_TIMEOUT << " seconds. If not online by the time of usage, the execution engine or instruction engine will go into <hardware_error> state");
        ROS_WARN("<generate stud position> action server not online. Please make sure it is online before attempting to instruct any tasks");
    }
    if(!move_client_->waitForServer(ros::Duration(WAIT_FOR_SERVER_TIMEOUT)))
    {
        ROS_DEBUG_STREAM("Waiting for the move platform action server to become online timed out after " << WAIT_FOR_SERVER_TIMEOUT << " seconds. If not online by the time of usage, the execution engine or instruction engine will go into <hardware_error> state");
        ROS_WARN("<move platform> action server not online. Please make sure it is online before attempting utilize the platform");
    }
}

void ActionInterface::sendWeldGoal(string task_name)
{
    mission_ctrl_msgs::executeWeldGoal goal;
    goal.task_name = task_name;

    //check if server is connected:
    if(!weld_client_->isServerConnected())
    {
        ROS_ERROR_STREAM("Failed to send <execute weld> action goal. Action server not connected!");
        ee_->hardwareError();
        return;
    }

    weld_client_->sendGoal(
                goal,
                boost::bind(&ActionInterface::armFinishedCB, this, _1, _2),
                boost::bind(&ActionInterface::armActiveCB, this),
                boost::bind(&ActionInterface::armFeedbackCB, this, _1));

    ROS_INFO("Execute weld action goal send");
}

void ActionInterface::sendMoveGoal(mission_ctrl_msgs::movePlatformGoal goal)
{
    if(!move_client_->isServerConnected())
    {
        ROS_ERROR_STREAM("Failed to send <move platform> action goal. Action server not connected!");
        if(ee_ != NULL)
            ee_->hardwareError();
        if(ie_ != NULL)
            ie_->hardwareError();
        return;
    }

    move_client_->sendGoal(
                goal,
                boost::bind(&ActionInterface::platformFinishedCB, this, _1, _2),
                boost::bind(&ActionInterface::platformActiveCB, this),
                boost::bind(&ActionInterface::platformFeedbackCB, this, _1));

    ROS_INFO("Move platform action goal send");
}

void ActionInterface::sendTeachGoal(string task_name)
{
    if(!teach_client_->isServerConnected())
    {
        ROS_ERROR_STREAM("Failed to send <teach> action goal. Action server not connected!");
        ie_->hardwareError();
        return;
    }

    mission_ctrl_msgs::performTeachingGoal goal;
    goal.task_name = task_name;

    teach_client_->sendGoal(
                goal,
                boost::bind(&ActionInterface::teachFinishedCB, this, _1, _2),
                boost::bind(&ActionInterface::teachActiveCB, this),
                boost::bind(&ActionInterface::teachFeedbackCB, this, _1));

    ROS_INFO("Teach action goal send");
}

void ActionInterface::sendGenPosGoal(string task_name)
{
    if(!gen_pos_client_->isServerConnected())
    {
        ROS_ERROR_STREAM("Failed to send <generate stud position> action goal. Action server not connected!");
        ie_->hardwareError();
        return;
    }

    mission_ctrl_msgs::generateStudDistributionGoal goal;
    goal.task_name = task_name;

    gen_pos_client_->sendGoal(
                goal,
                boost::bind(&ActionInterface::genPosFinishedCB, this, _1, _2),
                boost::bind(&ActionInterface::genPosActiveCB, this),
                boost::bind(&ActionInterface::genPosFeedbackCB, this, _1));


    ROS_INFO("Generate stud positions action goal send");
}

void ActionInterface::cancelArmGoal()
{
    weld_client_->cancelAllGoals();
}

void ActionInterface::cancelPlatformGoal()
{
    move_client_->cancelAllGoals();
}

void ActionInterface::cancelTeachGoal()
{
    teach_client_->cancelAllGoals();
}

void ActionInterface::cancelGenPosGoal()
{
    gen_pos_client_->cancelAllGoals();
}

void ActionInterface::cancelInstrGoals()
{
    cancelGenPosGoal();
    cancelTeachGoal();
    cancelPlatformGoal();
}

void ActionInterface::cancelExecGoals()
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

void ActionInterface::teachFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::performTeachingResultConstPtr &result)
{
    switch(state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ie_->teachDone();
        break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
        ie_->goalCancelled();
        break;
    case actionlib::SimpleClientGoalState::ABORTED:
        ie_->teachFailed();
        break;
    case actionlib::SimpleClientGoalState::LOST:
        ie_->teachFailed();
        break;
    default:
        ie_->teachFailed();
        break;
    }
}

void ActionInterface::teachActiveCB()
{
}

void ActionInterface::teachFeedbackCB(const mission_ctrl_msgs::performTeachingFeedbackConstPtr &feedback)
{

}

void ActionInterface::genPosFinishedCB(const actionlib::SimpleClientGoalState &state, const mission_ctrl_msgs::generateStudDistributionResultConstPtr &result)
{
    switch(state.state_)
    {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
        ie_->genPosDone(result->positions);
        break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
        ie_->goalCancelled();
        break;
    case actionlib::SimpleClientGoalState::ABORTED:
        ie_->genPosFailed();
        break;
    case actionlib::SimpleClientGoalState::LOST:
        ie_->genPosFailed();
        break;
    default:
        ie_->genPosFailed();
        break;
    }
}

void ActionInterface::genPosActiveCB()
{

}

void ActionInterface::genPosFeedbackCB(const mission_ctrl_msgs::generateStudDistributionFeedbackConstPtr &feedback)
{

}
