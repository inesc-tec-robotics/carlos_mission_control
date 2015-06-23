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

class ExecutionEngine
{
    friend class ActionInterface;

public:
    enum ExecState
    {
        STOPPED,
        NAV,
        WELD,
        NAV_ERROR,
        WELD_ERROR,
        HW_ERROR,
        WAIT_CANCEL
    };

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
    void sendProgressUpdate();

    //action client interface:
    ActionInterface* aci_;

private:

    struct ExecStateMachine;
    boost::shared_ptr<ExecStateMachine> esm_;

    ExecutionEngine();

    static ExecutionEngine* instance_;

    std::map<std::string,bool> enabled_functions;

    //action clients:
    movePlatform_client* platform_client_;
    executeWeld_client* arm_client_;

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

};

#endif // EXECUTIONENGINE_HPP_
