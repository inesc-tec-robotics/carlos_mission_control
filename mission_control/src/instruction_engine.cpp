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
#include "hw_state_machine.hpp"
#include "mission_handler.hpp"
#include "instruction_engine.hpp"
#include "system_engine.hpp"
#include "UI_API.hpp"



namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;
using namespace boost::msm::front::euml;
using namespace std;

//static task params:
vector<string> InstructionEngine::tasks;
int InstructionEngine::task_n;

// execution state machine:
namespace
{
////////////////////////////
/// Events
////////////////////////////

//user events
struct start_e {};
struct abort_e {};
struct retry_e{};
struct skip_task_e{};

//system events
struct nav_done_e{};
struct nav_fail_e{};
struct gen_done_e {};
struct teach_done_e {};
struct gen_fail_e {};
struct teach_fail_e {};
struct task_error_e {}; //event for other task errors than welding failed
struct hw_fail_e {};
struct cancelled_e{};

//////////////
///STATES
/// //////////

struct idle_s : public msm::front::state<>
{
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        cout << "initializing task" << endl;
    }
};
struct generate_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Entered stud generation state");
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Stud generation for task " << InstructionEngine::getCurrentTask() << " finished");
    }
};
struct teach_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        ROS_INFO_STREAM("Entered teach state");

        //call prodisp!

    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Teaching now done.");
    }
};
struct wait_cancel_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {

        cout << "waiting for cancel" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        cout << "now cancelled!" << endl;
    }
};
struct stopped_s : public msm::front::state<>
{
    template <class Event,class FSM>
    void on_entry(Event const& event ,FSM&)
    {
        //update mission state:
        if(MissionHandler::getInstance()->isLoaded())
            MissionHandler::getInstance()->updateMissionState();

        //Send signal to system engine
        SystemEngine::getInstance()->instructDone();
        ROS_INFO("Instruction engine now stopped.");
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
    }
};
struct hw_error_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {

        cout << "hardware error - call operator" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        cout << "operator has now confirmed!" << endl;
    }
};
struct gen_error_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Error in generating stud positions - call operator");
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Error now handled");
    }
};
struct teach_error_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Error in teaching - call operator");
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Error now handled");
    }
};
struct nav_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Navigate state");
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Navigation finished");
    }
};
struct nav_error_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Error in navigation - call operator");
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Error now handled");
    }
};
////////////////////////////
/// Transition actions
////////////////////////////
struct increment_task_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        //update task state:
        //MissionHandler::getInstance()->updateTaskState(tasks[task_n]);

        //increment the task iterator
        //task_n++;
    }
};
struct set_remaining_studs_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        //setting remaining studs to "failed"
    }
};
struct reset_stud_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        //resetting stud to "pending"
    }
};
struct reset_task_counter_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        //task_n = 0;
    }
};
struct prepare_instruction_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
//        //update task list:
//        tasks = MissionHandler::getInstance()->getExecutableTasks();

//        //reset the task counter:
//        task_n = 0;
    }
};
struct cancel_goals_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {

    }
};

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
struct more_tasks_g
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    bool operator()(EVT const& evt,FSM&,SourceState& ,TargetState& )
    {
//        if(task_n == (int)tasks.size()-1)
//            return false;
//        else
//            return true;
    }
};
struct mission_instructable_g
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    bool operator()(EVT const& evt,FSM&,SourceState& ,TargetState& )
    {
        if(!MissionHandler::getInstance()->isLoaded())
        {
            ROS_ERROR("Cannot instruct mission. No mission currently loaded.");
            return false;
        }
        mission::state state = MissionHandler::getInstance()->getState();
        if(state == mission::CONFIGURED || state == mission::PARTIALLY_INSTRUCTED)
        {
            return true;
        }
        else
            return false;
    }
};
struct task_gen_done_g
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    bool operator()(EVT const& evt,FSM&,SourceState& ,TargetState& )
    {
        //check if the task has already generated studs (task state will be "instructed")
        if(MissionHandler::getInstance()->getTaskState(InstructionEngine::getCurrentTask()) == mission::INSTRUCTED)
            return true;

        return false;
    }
};

// front-end: define the FSM structure
struct InstructionStateMachine_ : public msm::front::state_machine_def<InstructionStateMachine_>
{
    template <class Event,class FSM>
    void on_entry(Event const& ,FSM&)
    {
        ROS_INFO_STREAM("Instruction engine ready");
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Instruction engine stopped");
    }

    ///SELECT INITIAL STATE
    typedef stopped_s initial_state;

    // Transition table for player
    struct transition_table : mpl::vector<
            //    Start                 Event               Next            Action                      Guard
            //  +------------------     +---------------+---------------+---------------------+----------------------+
            //Row < stopped_s,            start_e,            teach_s,        prepare_instruction_a,      And_<hw_idle_g, mission_instructable_g >     >,
            //Row < stopped_s,            start_e,            generate_s,     prepare_instruction_a,      And_<hw_idle_g, mission_instructable_g, Not_<task_gen_done_g> >     >,
            Row < stopped_s,            start_e,            nav_s,          none,                       And_<hw_idle_g, mission_instructable_g >                >,
            Row < stopped_s,            hw_fail_e,          hw_error_s,     none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < nav_s,                nav_done_e,         generate_s,     none,                       none                >,
            Row < nav_s,                nav_fail_e,         nav_error_s,    none,                       none                >,
            Row < nav_s,                hw_fail_e,          hw_error_s,     none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < generate_s,           gen_done_e,         teach_s,        none,                       none                >,
            Row < generate_s,           gen_fail_e,         gen_error_s,    none,                       none                >,
            Row < generate_s,           abort_e,            wait_cancel_s,  cancel_goals_a,             none                >,
            Row < generate_s,           hw_fail_e,          hw_error_s,     none,                       none                >,

            //  +---------+-------------+---------+---------------------+----------------------+
            Row < teach_s,              teach_done_e,       stopped_s,      none,                       none                >, //after execution of last task
            Row < teach_s,              teach_done_e,       nav_s,          increment_task_a,           more_tasks_g        >,
            Row < teach_s,              teach_fail_e,       teach_error_s,  none,                       none                >,
            Row < teach_s,              abort_e,            wait_cancel_s,  cancel_goals_a,             none                >,
            Row < teach_s,              hw_fail_e,          hw_error_s,     none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < nav_error_s,          retry_e,            nav_s,          none,                       hw_idle_g           >,
            Row < nav_error_s,          skip_task_e,        stopped_s,      none,                       none                >,
            Row < nav_error_s,          skip_task_e,        nav_s,          increment_task_a,           more_tasks_g        >,
            Row < nav_error_s,          abort_e,            stopped_s,      none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < gen_error_s,          retry_e,            generate_s,     none,                       hw_idle_g           >,
            Row < gen_error_s,          skip_task_e,        stopped_s,      set_remaining_studs_a,      Not_<more_tasks_g>  >,
            Row < gen_error_s,          skip_task_e,        nav_s,          ActionSequence_< mpl::vector<set_remaining_studs_a, increment_task_a> >,     more_tasks_g  >, //jump to next task is more exists
            Row < gen_error_s,          abort_e,            stopped_s,      none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+

            //  +---------+-------------+---------+---------------------+----------------------+
            Row < wait_cancel_s,        cancelled_e,        stopped_s,      none,                       none                >
            > {};

    /* CS notes!!
     * Transition tables are evaluated from bottom to top!!
     */

    // Replaces the default no-transition response.
    template <class FSM,class Event>
    void no_transition(Event const& e, FSM&,int state)
    {
        ROS_ERROR_STREAM("No transition from state " << state << " on event " << typeid(e).name());
    }

};
// Pick a back-end
//typedef msm::back::state_machine<mode_machine_> ModeMachine;

}

// just inherit from back-end and this structure can be forward-declared in the header file
// for shorter compile-time
struct InstructionEngine::InstrStateMachine : public msm::back::state_machine<InstructionStateMachine_>
{
    InstrStateMachine() : msm::back::state_machine<InstructionStateMachine_>()
    {
    }
};

InstructionEngine* InstructionEngine::instance_ = NULL;

InstructionEngine* InstructionEngine::getInstance()
{
    if (!instance_)   // Only allow one instance of class to be generated.
        instance_ = new InstructionEngine();

    return instance_;
}

InstructionEngine::InstructionEngine()
{
    //task_n=0;
    //tasks = MissionHandler::getInstance()->getTaskList();

    ism_ = boost::shared_ptr<InstrStateMachine>(new InstructionEngine::InstrStateMachine());
    ism_->start();
}

// start the state machine (first call of on_entry)
bool InstructionEngine::start()
{
    return (ism_->process_event(start_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool InstructionEngine::abort()
{
    return (ism_->process_event(abort_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

string InstructionEngine::getCurrentTask()
{
    if(InstructionEngine::tasks.size() <= 0)
        return "none";

    return InstructionEngine::tasks[InstructionEngine::task_n];
}


