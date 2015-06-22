
#ifndef HWSTATEMACHINE_HPP_
#define HWSTATEMACHINE_HPP_

//=================================
// Forward declared dependencies
//=================================

//=================================
// Included dependencies
//=================================
#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "std_msgs/UInt8.h"
#include "boost/assign.hpp"
#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include "mission_ctrl_msgs/hardware_state.h"

namespace hardware
{
class state
{
public:
    void operator=( const std::string state_string )
    {
        for(std::map<hardware::states, std::string>::const_iterator it = compare_map_.begin();it!=compare_map_.end();it++)
        {
            if(state_string == it->second)
            {
                state_ = it->first;
                return;
            }
        }
        ROS_WARN_STREAM("Trying to assign unknown state: " << state_string << ". State will be set to HARDWARE::ERROR");
        state_ = hardware::ERROR;
    }

    void operator=( const hardware::states state )
    {
        state_ = state;
    }

    std::string toString() const
    {
        if(compare_map_.find(state_) == compare_map_.end())
            return "error";
        return compare_map_.find(state_)->second;
    }

    void setDescription(const std::string description)
    {
        description_ = description;
    }

    std::string getDescription() const
    {
        return description_;
    }

    inline bool operator==(const state& other) const { return (other.state_ == this->state_); }
    inline bool operator!=(const state& other) const { return !(*this == other); }
    inline bool operator==(const std::string& other) const { return (other == this->toString()); }
    inline bool operator!=(const std::string& other) const { return !(*this == other); }
    inline bool operator==(const hardware::states& other) const { return (other == this->state_); }
    inline bool operator!=(const hardware::states& other) const { return !(*this == other); }

private:
    hardware::states state_;
    std::string description_;

    std::map<hardware::states, std::string> compare_map_ = boost::assign::map_list_of
            (hardware::IDLE,            "idle")
            (hardware::BUSY,            "busy")
            (hardware::ERROR,           "error")
            (hardware::NOT_CONNECTED,   "not_connected");

};

} //end namespace

class HardwareStateMachine
{

public:
    static HardwareStateMachine* getInstance();

    ~HardwareStateMachine();

    hardware::state getRobotState(void) const { return this->robot_state_; }
    hardware::state getPlatformState(void) const { return this->platform_state_; }
    hardware::state getManipulatorState(void) const { return this->manipulator_state_; }

private:

    HardwareStateMachine();

    void platformStateCB(const mission_ctrl_msgs::hardware_state::ConstPtr &msg);
    void manipulatorStateCB(const mission_ctrl_msgs::hardware_state::ConstPtr &msg);

    void updateState();

    void platformTimeout(const ros::TimerEvent& timeout);
    void manipulatorTimeout(const ros::TimerEvent &timeout);

    static HardwareStateMachine* instance_;

    hardware::state robot_state_;
    hardware::state platform_state_;
    hardware::state manipulator_state_;


    ros::NodeHandle n;
    ros::Subscriber platform_state_sub;
    ros::Subscriber manipulator_state_sub;
    ros::Timer platform_timer_;
    ros::Timer manipulator_timer_;

    double state_input_frequency_;
};

#endif /* HWSTATEMACHINE_HPP_ */
