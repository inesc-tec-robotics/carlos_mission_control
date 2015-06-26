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
#include "math.h"
#include "cmath"
#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "hw_state_machine.hpp"
#include "mission_handler.hpp"
#include "instruction_engine.hpp"
#include "system_engine.hpp"
#include "action_interface.hpp"
#include "UI_API.hpp"
#include "mission_control/function_defines.h"



namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;
using namespace boost::msm::front::euml;
using namespace std;

// Instruction state machine:
namespace
{
////////////////////////////
/// Events
////////////////////////////

//user events
struct start_e {};
struct pause_e {};
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
struct generate_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::GEN_POS;
        InstructionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of(INSTR_ABORT) );

        InstructionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Instruction engine entering stud generation state");
        ROS_INFO_STREAM("Stud generation for task " << InstructionEngine::getInstance()->getCurrentTask() << " started");

        InstructionEngine::getInstance()->aci_->sendGenPosGoal(InstructionEngine::getInstance()->getCurrentTask());
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Stud generation for task " << InstructionEngine::getInstance()->getCurrentTask() << " finished");
        ROS_DEBUG("Instruction engine leaving stud generation state");
    }
};
struct teach_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::TEACH;
        InstructionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                              (INSTR_ABORT) );

        InstructionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Instruction engine entered teaching state.");
        ROS_INFO_STREAM("Teaching task " << InstructionEngine::getInstance()->getCurrentTask() << " started");

        //send goal
        InstructionEngine::getInstance()->aci_->sendTeachGoal(InstructionEngine::getInstance()->getCurrentTask());

    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Teaching now done.");
        ROS_DEBUG("Instruction engine leaving teaching state");
    }
};
struct nav_s : public msm::front::state<>
{
    template <class Event,class FSM>
    void on_entry(Event const& ,FSM&)
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::NAV;
        InstructionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                              (INSTR_ABORT)
                                                              (EXEC_PAUSE) );

        InstructionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Instruction engine entered navigation state.");
        ROS_INFO_STREAM("Navigating to task " << InstructionEngine::getInstance()->getCurrentTask() << ".");

        //get goal data:
        TaskParams task_params = MissionHandler::getInstance()->getTaskParams(InstructionEngine::getInstance()->getCurrentTask());

        //create goal:
        mission_ctrl_msgs::movePlatformGoal goal;
        goal.nav_goal = InstructionEngine::getInstance()->convert2PoseStamped(task_params.navigation_goal.x, task_params.navigation_goal.y, task_params.navigation_goal.yaw);

        //send goal
        InstructionEngine::getInstance()->aci_->sendMoveGoal(goal);
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Navigation to task " << InstructionEngine::getInstance()->getCurrentTask() << " finished.");
        ROS_DEBUG("Instruction engine exiting navigation state");
    }
};
struct wait_cancel_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::WAIT_CANCEL;
        vector<string> temp;
        InstructionEngine::getInstance()->setEnabledFunctions(temp);
        InstructionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Instruction engine entering wait for cancel state");
        ROS_INFO("Instruction engine waiting for goal cancel.");
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO("Goal cancelled");
        ROS_DEBUG("Instruction engine leaving wait for cancelled state");
    }
};
struct stopped_s : public msm::front::state<>
{
    template <class Event,class FSM>
    void on_entry(Event const& event ,FSM&)
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::STOPPED;
        InstructionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of (INSTR_START) );
        InstructionEngine::getInstance()->sendProgressUpdate("Instruction stopped and ready");

        //update mission state:
        if(MissionHandler::getInstance()->isLoaded())
            MissionHandler::getInstance()->updateMissionState();

        //Send signal to system engine
        SystemEngine::getInstance()->instructDone();

        ROS_DEBUG("Instruction engine entering stopped state");
        ROS_INFO("Instruction engine stopped");
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO("Instruction engine started");
        ROS_DEBUG("Instruction engine leaving stopped state");
    }
};
struct hw_error_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::HW_ERROR;
        InstructionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                              (INSTR_ABORT)
                                                              (INSTR_RETRY) );
        InstructionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Instruction engine entering hardware error state");
        ROS_INFO("Hardware error - call operator");
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO("Hardware error solved");
        ROS_DEBUG("Instruction engine leaving hardware error state");
    }
};
struct gen_error_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::GEN_POS_ERROR;
        InstructionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                              (INSTR_ABORT)
                                                              (INSTR_RETRY)
                                                              (INSTR_SKIP_TASK) );
        InstructionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Instruction engine entering generate position error state");
        ROS_INFO_STREAM("Generate stud position error in task " << InstructionEngine::getInstance()->getCurrentTask());
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_DEBUG("Instruction engine leaving generate position error state");
        ROS_INFO("Generate position error resolved");
    }
};
struct teach_error_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::TEACH_ERROR;
        InstructionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                              (INSTR_ABORT)
                                                              (INSTR_RETRY)
                                                              (INSTR_SKIP_TASK) );
        InstructionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Instruction engine entering teaching error state");
        ROS_INFO_STREAM("Teaching error in task " << InstructionEngine::getInstance()->getCurrentTask());
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_DEBUG("Instruction engine leaving teaching error state");
        ROS_INFO("Teaching error resolved");
    }
};
struct nav_error_s : public msm::front::state<>
{
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        InstructionEngine::getInstance()->current_state_ = InstrState::NAV_ERROR;
        InstructionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                              (INSTR_ABORT)
                                                              (INSTR_RETRY)
                                                              (INSTR_SKIP_TASK) );
        InstructionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Instruction engine entering navigation error state");
        ROS_INFO_STREAM("Navigation error in task " << InstructionEngine::getInstance()->getCurrentTask());
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_DEBUG("Instruction engine leaving navigation error state");
        ROS_INFO("Navigation error resolved");
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
        MissionHandler::getInstance()->updateTaskState(InstructionEngine::getInstance()->getCurrentTask());

        //increment the task iterator
        InstructionEngine::getInstance()->task_n++;
    }
};
struct prepare_instruction_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        //        //update task list:
        InstructionEngine::getInstance()->tasks = MissionHandler::getInstance()->getInstructableTasks();

        //        //reset the task counter:
        InstructionEngine::getInstance()->task_n = 0;
    }
};
struct cancel_goals_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        InstructionEngine::getInstance()->aci_->cancelInstrGoals();
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
        if(MissionHandler::getInstance()->getTaskState(InstructionEngine::getInstance()->getCurrentTask()) == mission::INSTRUCTED)
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
            Row < stopped_s,            start_e,            nav_s,          prepare_instruction_a,      And_<hw_idle_g, mission_instructable_g >                >,
            Row < stopped_s,            hw_fail_e,          hw_error_s,     none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < nav_s,                nav_done_e,         generate_s,     none,                       task_gen_done_g >,          //else if stud gen HAS been done
            Row < nav_s,                nav_done_e,         generate_s,     none,                       Not_<task_gen_done_g> >,    //if stud generation NOT done
            Row < nav_s,                nav_fail_e,         nav_error_s,    none,                       none                >,
            Row < nav_s,                hw_fail_e,          hw_error_s,     none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < generate_s,           gen_done_e,         teach_s,        none,                       none                >,
            Row < generate_s,           gen_fail_e,         gen_error_s,    none,                       none                >,
            Row < generate_s,           abort_e,            wait_cancel_s,  cancel_goals_a,             none                >,
            Row < generate_s,           hw_fail_e,          hw_error_s,     none,                       none                >,

            //  +---------+-------------+---------+---------------------+----------------------+
            Row < teach_s,              teach_done_e,       stopped_s,      none,                       none                >, //after instruction of last task
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
            Row < gen_error_s,          skip_task_e,        stopped_s,      none,                       Not_<more_tasks_g>  >,
            Row < gen_error_s,          skip_task_e,        nav_s,          increment_task_a,           more_tasks_g        >, //jump to next task is more exists
            Row < gen_error_s,          abort_e,            stopped_s,      none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < teach_error_s,        retry_e,            teach_s,        none,                       hw_idle_g           >,
            Row < teach_error_s,        skip_task_e,        stopped_s,      none,                       Not_<more_tasks_g>  >,
            Row < teach_error_s,        skip_task_e,        nav_s,          increment_task_a,           more_tasks_g        >, //jump to next task is more exists
            Row < teach_error_s,        abort_e,            stopped_s,      none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < wait_cancel_s,        cancelled_e,        stopped_s,      none,                       none                >
            > {};

    // Replaces the default no-transition response.
    template <class FSM,class Event>
    void no_transition(Event const& e, FSM&,int state)
    {
        ROS_ERROR_STREAM("No transition from state " << state << " on event " << typeid(e).name());
    }

};
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
    {
        instance_ = new InstructionEngine();
        instance_->init();
    }
    return instance_;
}

InstructionEngine::InstructionEngine()
{
}

void InstructionEngine::init()
{
    aci_ = new ActionInterface();
    aci_->initInstruction(this);

    ism_ = boost::shared_ptr<InstrStateMachine>(new InstructionEngine::InstrStateMachine());
    ism_->start();
}

void InstructionEngine::teachDone()
{
    ism_->process_event(teach_done_e());
}

void InstructionEngine::teachFailed()
{
    ism_->process_event(teach_fail_e());
}

void InstructionEngine::teachFeedback()
{

}

void InstructionEngine::genPosDone(vector<geometry_msgs::Point> stud_positions)
{
    //add the studs to the task
    for(int i=0;i<(int)stud_positions.size();i++)
    {
        stud_positions[i].x;
        MissionHandler::getInstance()->addStud(getCurrentTask(), stud_positions[i].x, stud_positions[i].y);
    }

    //update the task-state:
    MissionHandler::getInstance()->updateTaskState(getCurrentTask());

    //dispatch event to state machine
    ism_->process_event(gen_done_e());
}

void InstructionEngine::genPosFailed()
{
    ism_->process_event(gen_fail_e());
}

void InstructionEngine::genPosFeedback()
{

}

void InstructionEngine::goalCancelled()
{
    ism_->process_event(cancelled_e());
}

void InstructionEngine::hardwareError()
{
    ism_->process_event(hw_fail_e());
}

// start the state machine (first call of on_entry)
bool InstructionEngine::start()
{
    return (ism_->process_event(start_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool InstructionEngine::pause()
{
    return (ism_->process_event(pause_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool InstructionEngine::abort()
{
    return (ism_->process_event(abort_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool InstructionEngine::skipTask()
{
    return (ism_->process_event(skip_task_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool InstructionEngine::retry()
{
    return (ism_->process_event(retry_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

string InstructionEngine::getCurrentTask()
{
    if(tasks.size() <= 0)
        return "none";

    return tasks[task_n];
}

geometry_msgs::PoseStamped InstructionEngine::convert2PoseStamped(double x, double y, double yaw)
{
    geometry_msgs::PoseStamped nav_goal;

    //position
    nav_goal.pose.position.x = x;
    nav_goal.pose.position.y = y;

    //orientation
    double Ra = yaw;
    double Rb = 0.0;
    double Rc = 0.0;

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix(0,0) = std::cos(y) * std::cos(Rb);
    rotation_matrix(0,1) = std::cos(Ra) * std::sin(Rb) * std::sin(Rc) - std::sin(Ra) * std::cos(Rc);
    rotation_matrix(0,2) = std::cos(Ra) * std::sin(Rb) * std::cos(Rc) + std::sin(Ra) * std::sin(Rc);
    rotation_matrix(1,0) = std::sin(Ra) * std::cos(Rb);
    rotation_matrix(1,1) = std::sin(Ra) * std::sin(Rb) * std::sin(Rc) + std::cos(Ra) * std::cos(Rc);
    rotation_matrix(1,2) = std::sin(Ra) * std::sin(Rb) * std::cos(Rc) - std::cos(Ra) * std::sin(Rc);
    rotation_matrix(2,0) = -std::sin(Rb);
    rotation_matrix(2,1) = std::cos(Rb) * std::sin(Rc);
    rotation_matrix(2,2) = std::cos(Rb) * std::cos(Rc);

    Eigen::Quaternion<double> rot_quaternions(rotation_matrix);

    nav_goal.pose.orientation.w = rot_quaternions.w();
    nav_goal.pose.orientation.x = rot_quaternions.x();
    nav_goal.pose.orientation.y = rot_quaternions.y();
    nav_goal.pose.orientation.z = rot_quaternions.z();

    return nav_goal;
}

void InstructionEngine::setEnabledFunctions(std::vector<string> functions)
{
    enabled_functions = boost::assign::map_list_of
            (INSTR_START, false)
            (INSTR_ABORT, false)
            (INSTR_PAUSE, false)
            (INSTR_RETRY, false)
            (INSTR_SKIP_TASK, false);
    for(int i=0;i<(int)functions.size();i++)
    {
        enabled_functions[functions[i]] = true;
    }
}

void InstructionEngine::sendProgressUpdate(string description)
{
    mission_control::Progress progress;
    progress.engine_state = (string)current_state_;
    progress.current_mission = MissionHandler::getInstance()->getLoadedName();
    progress.current_task = getCurrentTask();

    if(description == "")   //no description added:
        description = progress.engine_state + " - " + progress.current_task;

    progress.description = description;

    for(map<string,bool>::iterator itr=enabled_functions.begin();itr!=enabled_functions.end();itr++)
    {
        mission_control::Function func;
        func.name = itr->first;
        func.enabled = itr->second;
        progress.enabled_functions.push_back(func);
    }

    UiAPI::getInstance()->instrProgressUpdate(progress);
}


