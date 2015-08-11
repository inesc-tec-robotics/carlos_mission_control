/* Created by Casper Schou @ AAU 2015
 *
 * This class holds a state machine to control the instruction of a mission.
 * It initally goes to the "stopped" state. Instruction start is triggered by the SystemEngine.
 * After instruction this state machine will transition into "stopped" state. Hence, it is NOT be
 * closed or deleted. It will stay in stopped state until again signaled to "start".
 *
 * This class implements a singelton pattern to make access to the same object from various classes easier
 *
 * This class simply holds the actual state machine (wraps it). The actual state machine
 * is implemented in the .cpp file.
 */

#ifndef INSTRUCTIONENGINE_HPP_
#define INSTRUCTIONENGINE_HPP_

//forward declaring ActionInterface, so that I can friend it.
class ActionInterface;

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "actionlib/client/simple_action_client.h"
#include "mission_ctrl_msgs/executeWeldAction.h"
#include "mission_ctrl_msgs/movePlatformAction.h"

/* Making a wrapper-class of the enum for instruction state.
 * This allows for inline conversion to/from unit8 and string + comparisons.
 */
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
    /* Friending class ActionInterface to allow it to call private functions.
     * This way, the non user-available functions can be kept private and thus
     * cannot be called from UIAPI.
     */
    friend class ActionInterface;

public:
    static InstructionEngine* getInstance();

    //get name of the task currently being processed.
    std::string getCurrentTask();

    //list of the tasks in the mission.
    std::vector<std::string> tasks;

    //index in the "tasks" vector currently being processed. (hence, points to the current task)
    int task_n;

    //current state:
    InstrState current_state_;

     //Event transition request functions called from UIAPI (thus, the user)
    bool start();
    bool pause();
    bool abort();
    bool skipTask();
    bool retry();

    //convert a 3x double into PoseStamped
    geometry_msgs::PoseStamped convert2PoseStamped(double x, double y, double yaw);

    /* Specify which functions are enabled
     * The InstructionEngine holds a list of "enabled functions".
     * This refers to the event-transitions currently allowable.
     * Direct relation to the Event transition request functions declared above.
     * Purpose of this list is for the UIAPI to provide this information to any GUI connected.
     */
    void setEnabledFunctions(std::vector<std::string> functions);

    /* Send a progress update to the UIAPI, which then sends it to connected
     * GUIs via a ROS topic.
     * The purpose is to inform UI's, that the progress information has changed.
     * Called from within the state machine.
     */
    void sendProgressUpdate(std::string description = "");

    //Add studs to task. Calls "MissionHandler::addStud" for each stud in stud_positions.
    void addStuds(std::vector<geometry_msgs::Point> stud_positions);

    /* action interface pointer
     * Because of the structure of the boost state machine, the ros action clients used to
     * send requests to the subsystems (platform, arm, prodisp) has been moved to a separate class.
     * The state machine is only intended to execute functions during transition - not while in state.
     * The state is merely a "flag". Thus, I can send the action goals from the state machine on transition,
     * but binding functions for receiving the result and feedback was a bit tricky. Most clean solution I
     * could find was to move the action clients to a separate class. The ActionInterface class is also used in
     * the ExectuionEngine.
     */
    ActionInterface* aci_;

private:

     //forward declaration of state machine
    struct InstrStateMachine;

    //pointer to state machine object. The actual state machine is defined in the .cpp file.
    boost::shared_ptr<InstrStateMachine> ism_;

    InstructionEngine();

    void init();

    static InstructionEngine* instance_;

    //Enabled functions. Map holds all functions, with a true/false signaling enabled/disabled.
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
