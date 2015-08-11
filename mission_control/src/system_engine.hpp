/* Created by Casper Schou @ AAU 2015
 *
 * This class holds a state machine to control the overall system state.
 * The "overall system state" referes to the state of the control system, hence
 * what "mode" the system is currently in; e.g. idle, executing, instructing etc.
 * The state machine is implemented using the Boost MSM library.
 *
 * The instruct and execute states triggers the InstructionEngine and the ExecutionEngine
 * to start processing the loaded mission. Therefore, once entering these states, the system
 * state machine simply awaits the "done" signal from the "child" state machine.
 *
 * This class simply hold the actual state machine (wraps it). The actual state machine
 * is implemented in the .cpp file.
 */


#ifndef SYSTEMENGINE_HPP_
#define SYSTEMENGINE_HPP_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/assign.hpp>
#include <map>

class SysState
{

public:
    enum values
    {
        IDLE,
        EXECUTING,
        INSTRUCTING,
        ASSISTING,
        EDITING
    };

    SysState()
    {
        compare_map_ = boost::assign::map_list_of
                (IDLE,                   "idle")
                (EXECUTING,              "executing")
                (INSTRUCTING,            "instructing")
                (ASSISTING,              "assisting")
                (EDITING,                "editing");
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
        std::cout << "Trying to assign unknown value: " << value_string << ". Value cannot be set!" << std::endl;
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


    inline bool operator==(const SysState& other) const { return (other.value_ == this->value_); }
    inline bool operator!=(const SysState& other) const { return !(*this == other); }
    inline bool operator==(const std::string& other) const { return (other == this->toString()); }
    inline bool operator!=(const std::string& other) const { return !(*this == other); }
    inline bool operator==(const values& other) const { return (other == this->value_); }
    inline bool operator!=(const values& other) const { return !(*this == other); }


private:
    values value_;

    std::map<values, std::string> compare_map_;
};

class SystemEngine
{
public:
    static SystemEngine* getInstance();

    /* Send an event request - hence, request transition into a given state.
     * These functions are called by the UIAPI, thus the events comes from  the user.
     */
    bool execute();
    bool instruct();
    bool assist();
    bool edit();

    /* Events signaling the completion of a given state.
     * These functions are called by the various state machines of the subsystem,
     * and thus are not coming from the user.
     */
    void executeDone();
    void instructDone();
    void assistDone();

    /* editDone does infact come from the user. The edit mode differs from the
     * other states, in that no state machine is started.
     * Edit mode is simply a "flag" used to allow user changes to mission data.
     * It is implemented as a state, so that any GUI interaction with the MC can
     * "lock" the system in edit mode. Hereby, should multiple GUI be connected at once,
     * one does not start execution while the other is waiting for user input.
     */
    bool editDone();

    //get flags from state machine:
    bool isMissionLocked() const;
    bool isEditAllowed() const;

    //current state:
    SysState current_state_;

private:

    struct SysStateMachine;

    //pointer to state machine object.
    boost::shared_ptr<SysStateMachine> ssm_;

    SystemEngine();
    ~SystemEngine();

    void init();

    static SystemEngine* instance_;

};

#endif // SYSTEMENGINE_HPP_
