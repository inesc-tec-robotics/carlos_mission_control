#ifndef EXECUTIONENGINE_HPP_
#define EXECUTIONENGINE_HPP_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "actionlib/client/simple_action_client.h"
#include "mission_ctrl_msgs/executeWeldAction.h"
#include "mission_ctrl_msgs/movePlatformAction.h"

typedef actionlib::SimpleActionClient<mission_ctrl_msgs::movePlatformAction> movePlatform_client;
typedef actionlib::SimpleActionClient<mission_ctrl_msgs::executeWeldAction> executeWeld_client;

//struct ExecutionProgress
//{
//    std::string mission;
//    std::string task;
//    std::string stud;

//};


class ExecutionEngine
{
    friend class ActionInterface;

public:
    static ExecutionEngine* getInstance();
    static std::string getCurrentTask();
    static std::vector<std::string> tasks;
    static int task_n;

    // event methods
    bool start();
    bool pause();
    bool abort();
    bool skipStud();
    bool skipTask();
    bool retry();

    geometry_msgs::PoseStamped convert2PoseStamped(double x, double y, double yaw);

private:

    struct ExecStateMachine;
    boost::shared_ptr<ExecStateMachine> esm_;

    ExecutionEngine();

    static ExecutionEngine* instance_;



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
