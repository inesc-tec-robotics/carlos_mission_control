#include "ros/ros.h"
#include "hw_state_machine.hpp"


#define HB_TIMEOUT_MULTIPLIER 5           //multiplier for heartbeat timeout

using namespace std;

HardwareStateMachine* HardwareStateMachine::instance_ = NULL;

HardwareStateMachine* HardwareStateMachine::getInstance()
{
    if (!instance_)   // Only allow one instance of class to be generated.
        instance_ = new HardwareStateMachine();

    return instance_;
}

HardwareStateMachine::HardwareStateMachine()
{
    //get frequency:
    if(!ros::param::get(CARLOS_FSM_FREQUENCY, this->state_input_frequency_))
        state_input_frequency_ = DEFAULT_STATE_FREQ;

    //initialise states:
    platform_state_ = hardware::NOT_CONNECTED;
    manipulator_state_ = hardware::NOT_CONNECTED;
    robot_state_ = hardware::ERROR;

    //start ros subscribers:
    platform_state_sub = n.subscribe(CARLOS_BASE_STATE_MSG, 10, &HardwareStateMachine::platformStateCB,this);
    manipulator_state_sub = n.subscribe(CARLOS_ARM_STATE_MSG, 10, &HardwareStateMachine::manipulatorStateCB,this);

    //create timers:
    float timeout = (1/state_input_frequency_) * HB_TIMEOUT_MULTIPLIER;
    manipulator_timer_ = n.createTimer(ros::Duration(timeout), &HardwareStateMachine::manipulatorTimeout, this, true); //"true" means this is a oneshot timer (only runs once - unless start is called again.)
    platform_timer_ = n.createTimer(ros::Duration(timeout), &HardwareStateMachine::platformTimeout, this, true);

    //stop timers (we will start them once we have received the first state)
    manipulator_timer_.stop();
    platform_timer_.stop();
}

HardwareStateMachine::~HardwareStateMachine()
{
//    manipulator_timer_->stop();
//    platform_timer_->stop();
//    delete manipulator_timer_;
//    delete platform_timer_;
}

void HardwareStateMachine::platformStateCB(const mission_ctrl_msgs::hardware_state::ConstPtr& msg)
{
    this->platform_state_ = static_cast<hardware::states>(msg->state);
    this->platform_state_.setDescription(msg->description);

    //update the timer:
    platform_timer_.stop();
    platform_timer_.start();

    updateState();
}

void HardwareStateMachine::manipulatorStateCB(const mission_ctrl_msgs::hardware_state::ConstPtr& msg)
{
    this->manipulator_state_ = static_cast<hardware::states>(msg->state);
    this->manipulator_state_.setDescription(msg->description);

    //update the timer:
    manipulator_timer_.stop();
    manipulator_timer_.start();

    updateState();
}

void HardwareStateMachine::updateState()
{
    if(platform_state_ == hardware::ERROR || manipulator_state_ == hardware::ERROR)                         //if 1 or more is in error      --> error
        robot_state_ = hardware::ERROR;
    else if(platform_state_ == hardware::NOT_CONNECTED || manipulator_state_ == hardware::NOT_CONNECTED)    //if 1 or more is not connected --> error
        robot_state_ = hardware::ERROR;
    else if(platform_state_ == hardware::BUSY || manipulator_state_ == hardware::BUSY)                      //if 1 or more is busy          --> busy
        robot_state_ = hardware::BUSY;
    else if(platform_state_ == hardware::IDLE && manipulator_state_ == hardware::IDLE)                      //if both is idle               --> idle
        robot_state_ = hardware::IDLE;
    else
    {
        ROS_WARN_STREAM("Updating state, but cannot interpret states. This shouldn't happen!" <<
                        " State of platform: " << platform_state_.toString() <<
                        " State of manipulator: " << manipulator_state_.toString());
        robot_state_ = hardware::ERROR;                                                                     //if unknown/not defined        --> error
    }

    //used for debug:
    //cout << "state: " << robot_state_.toString() << endl;
}

void HardwareStateMachine::manipulatorTimeout(const ros::TimerEvent& timeout)
{
    ROS_WARN_STREAM("Didn't receive a state from the manipulator sub-system in time. Manipulator state changed to <not_connected>. ");
    manipulator_state_ = hardware::NOT_CONNECTED;
    updateState();
}

void HardwareStateMachine::platformTimeout(const ros::TimerEvent& timeout)
{
    ROS_WARN_STREAM("Didn't receive a state from the platform sub-system in time. Platform state changed to <not_connected>. ");
    platform_state_ = hardware::NOT_CONNECTED;
    updateState();
}
