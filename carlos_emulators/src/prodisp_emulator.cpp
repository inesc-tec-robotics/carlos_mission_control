
//
//  main.cpp
//
//  Created by Casper Schou @ AAU on 03/06/2015
//  Copyright (c) 2015 Casper Schou. All rights reserved.
//

#include "ros/ros.h"
#include "ros/node_handle.h"
#include "actionlib/server/simple_action_server.h"
#include "mission_ctrl_msgs/performTeachingAction.h"
#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include "mission_ctrl_msgs/hardware_state.h"


using namespace std;

//declare action server:
actionlib::SimpleActionServer<mission_ctrl_msgs::performTeachingAction>* teach_server_;

void teachActionCB(const mission_ctrl_msgs::performTeachingGoalConstPtr& goal)
{
    mission_ctrl_msgs::performTeachingResult result;

    cout << "Please select outcome: " << endl;

    while(true)
    {
        string input;
        cout << "-----------Menu-----------" << endl;
        cout << "0      success" << endl;
        cout << "1      teaching failed" << endl;
        cout << "2      hw error"<< endl;
        cout << endl;
        cout << "choice: ";
        cin >> input;

        if(input == "0")
        {
            result.succeeded = true;
            teach_server_->setSucceeded(result);
            return;
        }
        else if(input == "1")
        {
            result.succeeded = false;
            result.description = "teaching task failed";
            teach_server_->setAborted(result);
            return;
        }
        else if(input == "2")
        {
            result.succeeded = false;
            result.description = "hardware error";
            teach_server_->setAborted(result);
            return;
        }
        else
        {
            cout << "invalid choice, try again..." << endl;
        }

    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "prodisp_emulator");

    ros::NodeHandle n;

    cout << "CARLOS Prodisp emulator started" << endl;

    cout << "starting action server..." << endl;

    teach_server_ = new actionlib::SimpleActionServer<mission_ctrl_msgs::performTeachingAction>(n, CARLOS_TEACHING_ACTION, boost::bind(&teachActionCB, _1), false);
    teach_server_->start();

    cout << "action server started. Ready to teach some crazy tasks" << endl;

    ros::spin();
    ros::waitForShutdown();

    return 0;
}
