/* Created by Casper Schou @ AAU 2015
 *
 * This class keeps track of the hardware state of each sub-system.
 * From the state of each hardware component, an overall system hardware
 * state is deducted.
 * Despite being named HardwareStateMachine, it does not implement any
 * state machine libraries.
 *
 * This class implements a singelton pattern to make access to the same object from various classes easier
 */

#ifndef HWSTATEMACHINE_HPP_
#define HWSTATEMACHINE_HPP_

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

/* Making a wrapper-class of the enum for hardware state.
 * This allows for inline conversion to/from unit8 and string + comparisons.
 */
namespace hardware
{
class state
{
public:

    state()
    {
        compare_map_ = boost::assign::map_list_of
                    (hardware::IDLE,            "idle")
                    (hardware::BUSY,            "busy")
                    (hardware::ERROR,           "error")
                    (hardware::NOT_CONNECTED,   "not_connected");
    }

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

    std::map<hardware::states, std::string> compare_map_;

};

} //end namespace

class HardwareStateMachine
{

public:
    static HardwareStateMachine* getInstance();

    ~HardwareStateMachine();

    //Get overall system hardware state (combined state)
    hardware::state getRobotState(void) const { return this->robot_state_; }

    //Get platform hardware state
    hardware::state getPlatformState(void) const { return this->platform_state_; }

    //Get manipulator hardware state
    hardware::state getManipulatorState(void) const { return this->manipulator_state_; }

private:

    HardwareStateMachine();

    //Callback functions for subscription to the topics with the incoming hardware states from the sub-systems
    void platformStateCB(const mission_ctrl_msgs::hardware_state::ConstPtr &msg);
    void manipulatorStateCB(const mission_ctrl_msgs::hardware_state::ConstPtr &msg);

    //Update the overall hardware state (deduced from the state of each sub-system)
    void updateState();

    /* Callback function for hear beat timeout event.
     * Each hardware sub-system is expected to publish its
     * state at a given frequency defined in mission_ctrl_defines.h
     * For each msg received, the respective timer is reset.
     * Should it timeout (and thus this function gets called), it
     * indicates, that the given hardware sub-system might have shutdown or
     * otherwise be unavailable.
     */
    void platformTimeout(const ros::TimerEvent& timeout);
    void manipulatorTimeout(const ros::TimerEvent &timeout);

    static HardwareStateMachine* instance_;

    //local variables holding the state of each system
    hardware::state robot_state_;
    hardware::state platform_state_;
    hardware::state manipulator_state_;

    ros::NodeHandle n;

    //Subscribers to the incoming state topics:
    ros::Subscriber platform_state_sub;
    ros::Subscriber manipulator_state_sub;

    //Timers to keep track of the heartbeat frequency of each sub system.
    ros::Timer platform_timer_;
    ros::Timer manipulator_timer_;

    //expected incoming frequency of hardware states:
    double state_input_frequency_;
};

#endif /* HWSTATEMACHINE_HPP_ */
