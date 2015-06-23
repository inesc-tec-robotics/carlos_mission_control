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

public:
    static InstructionEngine* getInstance();

    static std::string getCurrentTask();
    static std::vector<std::string> tasks;
    static int task_n;

    // event methods
    bool start();
    bool abort();

    geometry_msgs::PoseStamped convert2PoseStamped(double x, double y, double yaw);

    //action client interface:
    ActionInterface* aci_;

private:

    struct InstrStateMachine;
    boost::shared_ptr<InstrStateMachine> ism_;

    InstructionEngine();

    static InstructionEngine* instance_;

};

#endif // INSTRUCTIONENGINE_HPP_
