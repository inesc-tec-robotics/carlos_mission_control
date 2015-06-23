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
#include "mission_control/Progress.h"
#include "hw_state_machine.hpp"
#include "mission_handler.hpp"
#include "action_interface.hpp"
#include "execution_engine.hpp"
#include "system_engine.hpp"
#include "UI_API.hpp"
#include "function_defines.hpp"

namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;
using namespace boost::msm::front::euml;
using namespace std;

//execution state machine:
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
struct skip_stud_e{};
struct skip_task_e{};

//system events
struct nav_done_e {};
struct mani_done_e {};
struct nav_fail_e {};
struct weld_fail_e {};
struct task_error_e {}; //event for other task errors than welding failed
struct hw_fail_e {};
struct goal_cancelled_e{};

////////////////////////////
/// Flags
////////////////////////////
struct startPossible_f{};
struct pausePossible_f{};
struct abortPossible_f{};
struct retryPossible_f{};
struct skipTaskPossible_f{};
struct skipStudPossible_f{};

//////////////
///STATES
/// //////////
struct nav_error_s : public msm::front::state<>
{
    typedef mpl::vector3<skipTaskPossible_f,
    retryPossible_f,
    abortPossible_f>      flag_list;

    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const&,FSM& )
    {
        ExecutionEngine::getInstance()->current_state_ = ExecutionEngine::NAV_ERROR;
        ExecutionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                            (EXEC_ABORT)
                                                            (EXEC_RETRY)
                                                            (EXEC_SKIP_TASK) );
        ExecutionEngine::getInstance()->sendProgressUpdate();

        cout << "error in navigation - call operator" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        cout << "operator has now confirmed!" << endl;
    }
};
struct weld_error_s : public msm::front::state<>
{
    typedef mpl::vector4<skipStudPossible_f,
    skipTaskPossible_f,
    retryPossible_f,
    abortPossible_f>      flag_list;

    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        ExecutionEngine::getInstance()->current_state_ = ExecutionEngine::WELD_ERROR;
        ExecutionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                            (EXEC_ABORT)
                                                            (EXEC_RETRY)
                                                            (EXEC_SKIP_TASK)
                                                            (EXEC_SKIP_STUD) );
        ExecutionEngine::getInstance()->sendProgressUpdate();

        cout << "error in manipulation - call operator" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        cout << "operator has now confirmed!" << endl;
    }
};
struct hw_error_s : public msm::front::state<>
{
    typedef mpl::vector2<retryPossible_f,abortPossible_f>      flag_list;
    // every (optional) entry/exit methods get the event passed.
    template <class Event,class FSM>
    void on_entry(Event const& event,FSM& )
    {
        ExecutionEngine::getInstance()->current_state_ = ExecutionEngine::HW_ERROR;
        ExecutionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                            (EXEC_ABORT)
                                                            (EXEC_RETRY) );
        ExecutionEngine::getInstance()->sendProgressUpdate();

        cout << "hardware error - call operator" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        cout << "operator has now confirmed!" << endl;
    }
};
struct nav_s : public msm::front::state<>
{
    typedef mpl::vector2<pausePossible_f,abortPossible_f>      flag_list;
    template <class Event,class FSM>
    void on_entry(Event const& ,FSM&)
    {
        ExecutionEngine::getInstance()->current_state_ = ExecutionEngine::NAV;
        ExecutionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                            (EXEC_ABORT)
                                                            (EXEC_PAUSE) );
        ExecutionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Execution engine entered navigation state.");
        ROS_INFO_STREAM("Navigating to task " << ExecutionEngine::getInstance()->getCurrentTask() << ".");

        //get goal data:
        TaskParams task_params = MissionHandler::getInstance()->getTaskParams(ExecutionEngine::getInstance()->getCurrentTask());

        //create goal:
        mission_ctrl_msgs::movePlatformGoal goal;
        goal.nav_goal = ExecutionEngine::getInstance()->convert2PoseStamped(task_params.navigation_goal.x, task_params.navigation_goal.y, task_params.navigation_goal.yaw);

        //send goal
        ExecutionEngine::getInstance()->aci_->sendMoveGoal(goal);
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Navigation to task " << ExecutionEngine::getInstance()->getCurrentTask() << " finished.");
        ROS_DEBUG("Execution engine exiting navigation state");
    }
};
struct mani_s : public msm::front::state<>
{
    typedef mpl::vector2<pausePossible_f,abortPossible_f>      flag_list;
    template <class Event,class FSM>
    void on_entry(Event const& ,FSM&)
    {
        ExecutionEngine::getInstance()->current_state_ = ExecutionEngine::WELD;
        ExecutionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of
                                                            (EXEC_ABORT)
                                                            (EXEC_PAUSE) );
        ExecutionEngine::getInstance()->sendProgressUpdate();

        ROS_DEBUG("Execution engine entered manipulation state.");
        ROS_INFO_STREAM("Welding of task " << ExecutionEngine::getInstance()->getCurrentTask() << " started.");

        //send goal
        ExecutionEngine::getInstance()->aci_->sendWeldGoal(ExecutionEngine::getInstance()->getCurrentTask());
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        ROS_INFO_STREAM("Welding of task " << ExecutionEngine::getInstance()->getCurrentTask() << " finished.");
        ROS_DEBUG("Execution engine exiting manipulation state.");
    }
};
struct wait_cancel_s : public msm::front::state<>
{
    template <class Event,class FSM>
    void on_entry(Event const& event ,FSM&)
    {
        ExecutionEngine::getInstance()->current_state_ = ExecutionEngine::WAIT_CANCEL;
        vector<string> temp;
        ExecutionEngine::getInstance()->setEnabledFunctions(temp);
        ExecutionEngine::getInstance()->sendProgressUpdate();


        ROS_INFO("Execution engine waiting for goal cancel.");
    }
};
struct stopped_s : public msm::front::state<>
{
    typedef mpl::vector1<startPossible_f>      flag_list;
    template <class Event,class FSM>
    void on_entry(Event const& event ,FSM&)
    {
        ExecutionEngine::getInstance()->current_state_ = ExecutionEngine::STOPPED;
        ExecutionEngine::getInstance()->setEnabledFunctions(boost::assign::list_of (EXEC_START) );
        ExecutionEngine::getInstance()->sendProgressUpdate();

        //update mission state:
        if(MissionHandler::getInstance()->isLoaded())
            MissionHandler::getInstance()->updateMissionState();

        //Send signal to system engine
        SystemEngine::getInstance()->executeDone();

        ROS_INFO("Execution engine now stopped.");
    }
    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
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
        MissionHandler::getInstance()->updateTaskState(ExecutionEngine::getInstance()->getCurrentTask());

        //increment the task iterator
        ExecutionEngine::getInstance()->task_n++;
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
        ExecutionEngine::getInstance()->task_n = 0;
    }
};
struct send_execute_done_a //not used currently
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        SystemEngine::getInstance()->executeDone();
    }
};
struct prepare_execution_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        //update task list:
        ExecutionEngine::getInstance()->tasks = MissionHandler::getInstance()->getExecutableTasks();

        //reset the task counter:
        ExecutionEngine::getInstance()->task_n = 0;
    }
};
struct cancel_goals_a
{
    template <class FSM,class EVT,class SourceState,class TargetState>
    void operator()(EVT const& ,FSM& fsm,SourceState& ,TargetState& )
    {
        ExecutionEngine::getInstance()->aci_->cancelExecGoals();
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
        if(ExecutionEngine::getInstance()->task_n == (int)ExecutionEngine::getInstance()->tasks.size()-1)
            return false;
        else
            return true;
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
            return false;
    }
};

// front-end: define the FSM structure
struct ExecutionStateMachine_ : public msm::front::state_machine_def<ExecutionStateMachine_>
{
    template <class Event,class FSM>
    void on_entry(Event const& ,FSM&)
    {
        cout << "Execution engine started" << endl;
    }

    template <class Event,class FSM>
    void on_exit(Event const&,FSM& )
    {
        cout << "Execution engine stopped" << endl;
    }

    ///SELECT INITIAL STATE
    typedef stopped_s initial_state;

    // Transition table for player
    struct transition_table : mpl::vector<
            //    Start                 Event               Next            Action                      Guard
            //  +------------------     +---------------+---------------+---------------------+----------------------+
            Row < stopped_s,            start_e,            nav_s,          prepare_execution_a,       And_<hw_idle_g, mission_executable_g>     >,
            //Row < stopped_s,            abort_e,            stopped_s,      none,                     none                >,
            Row < stopped_s,            hw_fail_e,          hw_error_s,     none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < nav_s,                nav_done_e,         mani_s,         none,                       none                >,
            Row < nav_s,                nav_fail_e,         nav_error_s,    none,                       none                >,
            Row < nav_s,                abort_e,            wait_cancel_s,  cancel_goals_a,             none                >,
            Row < nav_s,                hw_fail_e,          hw_error_s,     none,                       none                >,

            //  +---------+-------------+---------+---------------------+----------------------+
            Row < mani_s,               mani_done_e,        stopped_s,      none,                       none                >, //after execution of last task
            Row < mani_s,               mani_done_e,        nav_s,          increment_task_a,           more_tasks_g        >,
            Row < mani_s,               weld_fail_e,        weld_error_s,   none,                       none                >,
            Row < mani_s,               abort_e,            wait_cancel_s,  cancel_goals_a,             none                >,
            Row < mani_s,               hw_fail_e,          hw_error_s,     none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < weld_error_s,         retry_e,            mani_s,         reset_stud_a,               hw_idle_g           >,
            Row < weld_error_s,         skip_stud_e,        mani_s,         none,                       hw_idle_g           >,
            Row < weld_error_s,         skip_task_e,        stopped_s,      set_remaining_studs_a,      Not_<more_tasks_g>  >,
            Row < weld_error_s,         skip_task_e,        nav_s,          ActionSequence_< mpl::vector<set_remaining_studs_a, increment_task_a> >,     more_tasks_g  >, //jump to next task is more exists
            Row < weld_error_s,         abort_e,            stopped_s,      none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < nav_error_s,          retry_e,            nav_s,          none,                       hw_idle_g           >,
            Row < nav_error_s,          skip_task_e,        stopped_s,      none,                       none                >,
            Row < nav_error_s,          skip_task_e,        nav_s,          increment_task_a,           more_tasks_g        >,
            Row < nav_error_s,          abort_e,            stopped_s,      none,                       none                >,
            //  +---------+-------------+---------+---------------------+----------------------+
            Row < wait_cancel_s,        goal_cancelled_e,   stopped_s,      none,                       none                >
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
struct ExecutionEngine::ExecStateMachine : public msm::back::state_machine<ExecutionStateMachine_>
{
    ExecStateMachine() : msm::back::state_machine<ExecutionStateMachine_>()
    {
    }
};

ExecutionEngine* ExecutionEngine::instance_ = NULL;

ExecutionEngine* ExecutionEngine::getInstance()
{
    if (!instance_)   // Only allow one instance of class to be generated.
        instance_ = new ExecutionEngine();

    return instance_;
}

ExecutionEngine::ExecutionEngine()
{
}

void ExecutionEngine::init()
{
    aci_ = new ActionInterface();
    aci_->initExecution(this);

    esm_ = boost::shared_ptr<ExecStateMachine>(new ExecutionEngine::ExecStateMachine());
    esm_->start();
}

// start the state machine (first call of on_entry)
bool ExecutionEngine::start()
{
    return (esm_->process_event(start_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

// the public interfaces simply forwards events to the state machine
bool ExecutionEngine::pause()
{
    return (esm_->process_event(pause_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool ExecutionEngine::abort()
{
    return (esm_->process_event(abort_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool ExecutionEngine::skipStud()
{
    return (esm_->process_event(skip_stud_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool ExecutionEngine::skipTask()
{
    return (esm_->process_event(skip_task_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

bool ExecutionEngine::retry()
{
    return (esm_->process_event(retry_e()) == boost::msm::back::HANDLED_TRUE ? true : false);
}

void ExecutionEngine::maniDone()
{
    esm_->process_event(mani_done_e());
}

void ExecutionEngine::maniFailed()
{
    esm_->process_event(weld_fail_e());
}

void ExecutionEngine::maniActive()
{
}

void ExecutionEngine::maniFeedback()
{
    //send signal to UIAPI that there is an update:
    sendProgressUpdate();
}

void ExecutionEngine::navDone()
{
    esm_->process_event(nav_done_e());
}

void ExecutionEngine::navFailed()
{
    esm_->process_event(nav_fail_e());
}

void ExecutionEngine::navActive()
{

}

void ExecutionEngine::navFeedback()
{

}

void ExecutionEngine::goalCancelled()
{
    esm_->process_event(goal_cancelled_e());
}

string ExecutionEngine::getCurrentTask()
{
    if(tasks.size() <= 0)
        return "none";

    return tasks[task_n];
}

geometry_msgs::PoseStamped ExecutionEngine::convert2PoseStamped(double x, double y, double yaw)
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

void ExecutionEngine::sendProgressUpdate()
{
    mission_control::Progress progress;
    progress.engine_state = (unsigned int)current_state_;
    progress.current_mission = MissionHandler::getInstance()->getLoadedName();
    progress.current_task = getCurrentTask();

    for(map<string,bool>::iterator itr=enabled_functions.begin();itr!=enabled_functions.end();itr++)
    {
        mission_control::Function func;
        func.name = itr->first;
        func.enabled = itr->second;
        progress.enabled_functions.push_back(func);
    }

    UiAPI::getInstance()->execProgressUpdate(progress);
}

void ExecutionEngine::setEnabledFunctions(vector<string> functions)
{
    enabled_functions = boost::assign::map_list_of(EXEC_START, false)
            (EXEC_ABORT, false)
            (EXEC_PAUSE, false)
            (EXEC_RETRY, false)
            (EXEC_SKIP_STUD, false)
            (EXEC_SKIP_TASK, false);
    for(int i=0;i<(int)functions.size();i++)
    {
        enabled_functions[functions[i]] = true;
    }
}
