#ifndef INSTRUCTIONENGINE_HPP_
#define INSTRUCTIONENGINE_HPP_

class ActionInterface;

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "actionlib/client/simple_action_client.h"
#include "mission_ctrl_msgs/executeWeldAction.h"
#include "mission_ctrl_msgs/movePlatformAction.h"

class InstructionEngine
{
    friend class ActionInterface;

public:
    enum InstrState
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

public:
    static InstructionEngine* getInstance();

    void init();

    std::string getCurrentTask();
    std::vector<std::string> tasks;
    int task_n;

    //current state:
    InstrState current_state_;

    // event methods
    bool start();
    bool abort();

    geometry_msgs::PoseStamped convert2PoseStamped(double x, double y, double yaw);

    void setEnabledFunctions(std::vector<std::string> functions);
    void sendProgressUpdate();

    //action client interface:
    ActionInterface* aci_;

private:

    struct InstrStateMachine;
    boost::shared_ptr<InstrStateMachine> ism_;

    InstructionEngine();

    static InstructionEngine* instance_;

    std::map<std::string,bool> enabled_functions;

    //functions NOT offered to the user (only to my friend ActionInterface - he's a well-behaved guy!)
    void teachDone();
    void teachFailed();
    void teachFeedback();
    void genPosDone(std::vector<geometry_msgs::Point> stud_positions);
    void genPosFailed();
    void genPosFeedback();
    void goalCancelled();

};

#endif // INSTRUCTIONENGINE_HPP_
