#ifndef EXECUTIONENGINE_HPP_
#define EXECUTIONENGINE_HPP_

class ActionInterface;

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "actionlib/client/simple_action_client.h"
#include "mission_ctrl_msgs/executeWeldAction.h"
#include "mission_ctrl_msgs/movePlatformAction.h"

typedef actionlib::SimpleActionClient<mission_ctrl_msgs::movePlatformAction> movePlatform_client;
typedef actionlib::SimpleActionClient<mission_ctrl_msgs::executeWeldAction> executeWeld_client;

class ExecState
{
public:
    enum values
    {
        STOPPED,
        NAV,
        WELD,
        NAV_ERROR,
        WELD_ERROR,
        HW_ERROR,
        WAIT_CANCEL
    };

    ExecState()
    {
        compare_map_ = boost::assign::map_list_of
                (STOPPED,                   "stopped")
                (NAV,                       "navigating")
                (WELD,                      "welding")
                (NAV_ERROR,                 "navigation_error")
                (WELD_ERROR,                "welding_error")
                (HW_ERROR,                  "hardware_error")
                (WAIT_CANCEL,               "cancelling");
    }

    void operator=( const std::string value_string )
    {
        for(std::map<values, std::string>::const_iterator it = compare_map_.begin();it!=compare_map_.end();it++)
        {
            if(value_string == it->second)
            {
                value_ = it->first;
                return;
            }
        }
        ROS_WARN_STREAM("Trying to assign unknown value: " << value_string << ". Value cannot be set!");
    }

    void operator=( const values value )
    {
        value_ = value;
    }

    operator int() {return (int)value_;}
    operator unsigned int() {return (unsigned int)value_;}
    operator std::string() {return toString();}
    operator values() {return value_;}

    std::string toString() const
    {
        if(compare_map_.find(value_) == compare_map_.end())
            return "error";
        return compare_map_.find(value_)->second;
    }


    inline bool operator==(const ExecState& other) const { return (other.value_ == this->value_); }
    inline bool operator!=(const ExecState& other) const { return !(*this == other); }
    inline bool operator==(const std::string& other) const { return (other == this->toString()); }
    inline bool operator!=(const std::string& other) const { return !(*this == other); }
    inline bool operator==(const values& other) const { return (other == this->value_); }
    inline bool operator!=(const values& other) const { return !(*this == other); }


private:
    values value_;

    std::map<values, std::string> compare_map_;
};

class ExecutionEngine
{
    friend class ActionInterface;

public:
    static ExecutionEngine* getInstance();

    std::string getCurrentTask();
    std::vector<std::string> tasks;
    int task_n;

    //current state:
    ExecState current_state_;

    // event methods
    bool start();
    bool pause();
    bool abort();
    bool skipStud();
    bool skipTask();
    bool retry();

    geometry_msgs::PoseStamped convert2PoseStamped(double x, double y, double yaw);

    void setEnabledFunctions(std::vector<std::string> functions);
    void sendProgressUpdate(std::string description = "");

    //action client interface:
    ActionInterface* aci_;

private:

    struct ExecStateMachine;
    boost::shared_ptr<ExecStateMachine> esm_;

    ExecutionEngine();

    void init();

    static ExecutionEngine* instance_;

    std::map<std::string,bool> enabled_functions;

    //functions NOT offered to the user (only to my friend ActionInterface - he's a well-behaved guy!)
    void maniDone();
    void maniFailed();
    void maniActive();
    void maniFeedback();
    void navDone();
    void navFailed();
    void navActive();
    void navFeedback();
    void goalCancelled();
    void hardwareError();

};

#endif // EXECUTIONENGINE_HPP_
