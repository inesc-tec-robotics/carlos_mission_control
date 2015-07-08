#ifndef INSTRUCTIONENGINE_HPP_
#define INSTRUCTIONENGINE_HPP_

class ActionInterface;

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "actionlib/client/simple_action_client.h"
#include "mission_ctrl_msgs/executeWeldAction.h"
#include "mission_ctrl_msgs/movePlatformAction.h"

class InstrState
{
    friend class InstrStateMachine;

public:
    enum values
    {
        STOPPED,
        NAV,
        GEN_POS,
        TEACH,
        NAV_ERROR,
        GEN_POS_ERROR,
        TEACH_ERROR,
        HW_ERROR,
        WAIT_CANCEL
    };


    InstrState()
    {
        compare_map_ = boost::assign::map_list_of
                (STOPPED,                   "stopped")
                (NAV,                       "navigating")
                (GEN_POS,                   "generate_stud_postions")
                (TEACH,                     "teaching")
                (NAV_ERROR,                 "navigation_error")
                (GEN_POS_ERROR,             "generate_stud_pos_error")
                (TEACH_ERROR,               "teaching_error")
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


    inline bool operator==(const InstrState& other) const { return (other.value_ == this->value_); }
    inline bool operator!=(const InstrState& other) const { return !(*this == other); }
    inline bool operator==(const std::string& other) const { return (other == this->toString()); }
    inline bool operator!=(const std::string& other) const { return !(*this == other); }
    inline bool operator==(const values& other) const { return (other == this->value_); }
    inline bool operator!=(const values& other) const { return !(*this == other); }


private:
    values value_;

    std::map<values, std::string> compare_map_;
};

class InstructionEngine
{
    friend class ActionInterface;

public:
    static InstructionEngine* getInstance();

    std::string getCurrentTask();
    std::vector<std::string> tasks;
    int task_n;

    //current state:
    InstrState current_state_;

    // event methods
    bool start();
    bool pause();
    bool abort();
    bool skipTask();
    bool retry();

    geometry_msgs::PoseStamped convert2PoseStamped(double x, double y, double yaw);

    void setEnabledFunctions(std::vector<std::string> functions);
    void sendProgressUpdate(std::string description = "");

    //Add studs to task. Wraps the call to "mission handler - addStud".
    void addStuds(std::vector<geometry_msgs::Point> stud_positions);

    //action client interface:
    ActionInterface* aci_;

private:

    struct InstrStateMachine;
    boost::shared_ptr<InstrStateMachine> ism_;

    InstructionEngine();

    void init();

    static InstructionEngine* instance_;

    std::map<std::string,bool> enabled_functions;

    //functions NOT offered to the user (only to my friend ActionInterface - he's a well-behaved guy!)
    void teachDone();
    void teachFailed();
    void teachFeedback();
    void genPosDone(std::vector<geometry_msgs::Point> stud_positions);
    void genPosFailed();
    void genPosFeedback();
    void navDone();
    void navFailed();
    void goalCancelled();
    void hardwareError();
};

#endif // INSTRUCTIONENGINE_HPP_
