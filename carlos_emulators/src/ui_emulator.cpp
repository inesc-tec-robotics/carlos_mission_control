
#include "ros/ros.h"
#include "ros/node_handle.h"
#include "actionlib/server/simple_action_server.h"
#include "mission_ctrl_msgs/movePlatformAction.h"
#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include "mission_ctrl_msgs/hardware_state.h"


using namespace std;




int main(int argc, char * argv[])
{
    ros::init(argc, argv, "platform_emulator");

    ros::NodeHandle n;

    cout << "CARLOS UI emulator started" << endl;

    cout << "I don't do anything yet!" << endl;


    ros::spin();
    ros::waitForShutdown();

    return 0;
}
