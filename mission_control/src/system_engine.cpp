#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30 //increasing limit of max sice as needed in trans. table
#define BOOST_MPL_LIMIT_MAP_SIZE 30 //

#include <iostream>
// we have more than the default 20 transitions, so we need to require more from Boost.MPL
#include "boost/mpl/vector/vector30.hpp"
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>
#include "system_engine.hpp"
#include "mission_handler.hpp"
#include "hw_state_machine.hpp"
#include "execution_engine.hpp"

namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;
using namespace boost::msm::front::euml;
using namespace std;

// execution state machine:
namespace
{
////////////////////////////
/// Events
////////////////////////////
struct exec_start_e{};
struct exec_done_e{};
struct instruct_start_e{};
struct instruct_done_e{};
struct assist_start_e{};
struct assist_done_e{};
struct exit_e{};
struct shutdown_e{};

////////////////////////////
/// Flags
/// ///////////////////////
struct EditAllowed{}; //not used at the moment

//////////////
///STATES
/// //////////
struct idle_s : public msm::front::state<>
{
    typedef mpl::vector1<EditAllowed>      flag_list;
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        cout << "Entered idle state" << endl;
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
    }
};
struct executing_s : public msm::front::state<>
{
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        cout << "Entered executing state" << endl;
        ExecutionEngine::getInstance()->start();
        SystemEngine::getInstance()->lockMissionHandler();

    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        SystemEngine::getInstance()->unlockMissionHandler();
        cout << "Execution terminated" << endl;
    }
};
struct instructing_s : public msm::front::state<>
{
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        cout << "Entered instructing state" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
    }
};
struct assisting_s : public msm::front::state<>
{
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        cout << "Entered assisting state" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
    }
};

////////////////////////////
/// Transition actions
////////////////////////////

////////////////////////////
/// Transition guards
////////////////////////////
struct hw_idle_g
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    bool operator()(EVT const& evt,FSM&,SourceState& ,TargetState& )
    {
        //do check the actual hardware state!
        if(HardwareStateMachine::getInstance()->getRobotState() == hardware::IDLE)
            return true;

        return false;
    }
};
struct mission_executable_g
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    bool operator()(EVT const& evt,FSM&,SourceState& ,TargetState& )
    {
        if(!MissionHandler::getInstance()->isLoaded())
        {
            ROS_ERROR("Cannot execute mission. No mission currently loaded.");
            return false;
        }
        mission::state state = MissionHandler::getInstance()->getState();
        if(state == mission::INSTRUCTED || state == mission::PARTIALLY_COMPLETED)
        {
            return true;
        }
        else
        {
            if(state == mission::COMPLETED)
                ROS_ERROR_STREAM("Cannot execute mission " << MissionHandler::getInstance()->getLoadedName() << ". Mission already completed!");
            else
                ROS_ERROR_STREAM("Cannot execute mission " << MissionHandler::getInstance()->getLoadedName() << ". Not an executable mission!");

            return false;
        }
    }
};
struct mission_instructable_g
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    bool operator()(EVT const& evt,FSM&,SourceState& ,TargetState& )
    {
        if(MissionHandler::getInstance()->isLoaded())
        {
            ROS_ERROR("Cannot instruct mission. No mission currently loaded.");
            return false;
        }
        mission::state state = MissionHandler::getInstance()->getState();
        if(state != mission::CONFIGURED && state != mission::PARTIALLY_INSTRUCTED)      //should it be possible to instruct an already instructed mission?
        {
            return true;
        }
        else
        {
            if(state == mission::INSTRUCTED)
                ROS_ERROR_STREAM("Cannot execute mission " << MissionHandler::getInstance()->getLoadedName() << ". Mission already instructed!");
            else
                ROS_ERROR_STREAM("Cannot execute mission " << MissionHandler::getInstance()->getLoadedName() << ". Not an instructable mission!");

            return false;
        }
    }
};

// front-end: define the FSM structure
struct SystemStateMachine_ : public msm::front::state_machine_def<SystemStateMachine_>
{
    template <class Event,class FSM>
    void on_entry(Event const& ,FSM&)
    {
        cout << "System engine started" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        cout << "System engine stopped" << endl;
    }

    ///SELECT INITIAL STATE
    typedef idle_s initial_state;

    // Transition table for player
    struct transition_table : mpl::vector<
            //    Start                 Event               Next            Action                      Guard
            //  +------------------     +---------------+---------------+---------------------+----------------------+
            Row < idle_s,               exec_start_e,             executing_s,    none,                       And_<hw_idle_g, mission_executable_g>    >,
            Row < idle_s,               instruct_start_e,         instructing_s,  none,                       And_<hw_idle_g, mission_instructable_g>  >,
            Row < idle_s,               assist_start_e,           assisting_s,    none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < executing_s,          exit_e,             idle_s,         none,                       none                >,
            Row < executing_s,          exec_done_e,        idle_s,         none,                       none                >,
            Row < instructing_s,        exit_e,             idle_s,         none,                       none                >,
            Row < instructing_s,        instruct_done_e,    idle_s,         none,                       none                >,
            Row < assisting_s,          exit_e,             idle_s,         none,                       none                >,
            Row < assisting_s,          assist_done_e,      idle_s,         none,                       none                >
            > {};

    /* CS notes!!
     * Transition tables are evaluated from bottom to top!!
     */

    // Replaces the default no-transition response.
    template <class FSM,class Event>
    void no_transition(Event const& e, FSM&,int state)
    {
        //ROS_ERROR_STREAM("No transition from state " << state << " on event " << typeid(e).name());
    }

};

}

// just inherit from back-end and this structure can be forward-declared in the header file
// for shorter compile-time
struct SystemEngine::SysStateMachine : public msm::back::state_machine<SystemStateMachine_>
{
    SysStateMachine() : msm::back::state_machine<SystemStateMachine_>()
    {
    }
};

SystemEngine* SystemEngine::instance_ = NULL;

SystemEngine* SystemEngine::getInstance()
{
    if (!instance_)   // Only allow one instance of class to be generated.
        instance_ = new SystemEngine();

    return instance_;
}

SystemEngine::SystemEngine()
{
    ssm_ = boost::shared_ptr<SysStateMachine>(new SystemEngine::SysStateMachine());
    ssm_->start();

}

bool SystemEngine::execute()
{
    return(ssm_->process_event(exec_start_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

void SystemEngine::executeDone()
{
    ssm_->process_event(exec_done_e());
}

void SystemEngine::instructDone()
{
    ssm_->process_event(instruct_done_e());
}

void SystemEngine::assistDone()
{
    ssm_->process_event(assist_done_e());
}

void SystemEngine::lockMissionHandler()
{
    MissionHandler::getInstance()->lock();
}

void SystemEngine::unlockMissionHandler()
{
    MissionHandler::getInstance()->unlock();
}
