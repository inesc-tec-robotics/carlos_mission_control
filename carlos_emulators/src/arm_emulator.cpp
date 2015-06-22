
//
//  main.cpp
//
//  Created by Casper Schou @ AAU on 03/06/2015
//  Copyright (c) 2015 Casper Schou. All rights reserved.
//

#include "ros/ros.h"
#include "ros/node_handle.h"
#include "actionlib/server/simple_action_server.h"
#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include "mission_ctrl_msgs/hardware_state.h"
#include "mission_ctrl_msgs/executeWeldAction.h"

using namespace std;

//declare action server:
actionlib::SimpleActionServer<mission_ctrl_msgs::executeWeldAction>* as_;

//publisher
ros::Publisher state_publisher_;

hardware::states state_;

void actionCB(const mission_ctrl_msgs::executeWeldGoalConstPtr& goal)
{

    state_ = hardware::BUSY;

    mission_ctrl_msgs::executeWeldResult result;

    ROS_INFO_STREAM("Received a goal - execute task: " << goal->task_name);

    cout << "Please select outcome: " << endl;

    while(true)
    {
        string input;
        cout << "-----------Menu-----------" << endl;
        cout << "0      success" << endl;
        cout << "1      task failed" << endl;
        cout << "2      hw error"<< endl;
        cout << endl;
        cout << "choice: ";
        cin >> input;

        if(input == "0")
        {
            result.result_state = true;
            as_->setSucceeded(result);
            state_ = hardware::IDLE;
            return;
        }
        else if(input == "1")
        {
            result.result_state = false;
            result.error_string = "welding task failed";
            as_->setAborted(result);
            state_ = hardware::IDLE;
            return;
        }
        else if(input == "2")
        {
            result.result_state = false;
            result.error_string = "hardware error";
            as_->setAborted(result);
            state_ = hardware::ERROR;
            return;
        }
        else
        {
            cout << "invalid choice, try again..." << endl;
        }

    }
}

void statePubTimeout(const ros::TimerEvent& event)
{
    mission_ctrl_msgs::hardware_state state;
    state.state = state_;
    state.description = "this is a description";
    state_publisher_.publish(state);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "arm_emulator");

    ros::NodeHandle n;

    cout << "CARLOS Arm emulator started" << endl;

    cout << "starting action server..." << endl;

    as_ = new actionlib::SimpleActionServer<mission_ctrl_msgs::executeWeldAction>(n, CARLOS_WELD_ACTION, boost::bind(&actionCB, _1), false);
    as_->start();

    cout << "action server started. Ready to build some ships" << endl;

    //create publisher of states:
    state_ = hardware::IDLE;
    state_publisher_ = n.advertise<mission_ctrl_msgs::hardware_state>(CARLOS_ARM_STATE_MSG,10);

    //create timer for state publishing:
    //try to get the frequency from param server:
    double freq = DEFAULT_STATE_FREQ;
    if(ros::param::has(CARLOS_FSM_FREQUENCY))
        ros::param::get(CARLOS_FSM_FREQUENCY,freq);

    ros::Timer state_pub_timer_ = n.createTimer(ros::Duration((1/freq)),statePubTimeout );

    ros::spin();
    ros::waitForShutdown();

    return 0;
}
