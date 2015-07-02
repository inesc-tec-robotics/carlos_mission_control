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
#include "mission_ctrl_msgs/generateStudDistributionAction.h"
#include "text_color_defines.h"

#define WALL_HEIGHT 0.4  //2.5
#define WALL_WIDTH 0.4  //0.7

using namespace std;

//declare action server:
actionlib::SimpleActionServer<mission_ctrl_msgs::executeWeldAction>* weld_server_;
actionlib::SimpleActionServer<mission_ctrl_msgs::generateStudDistributionAction>* gen_pos_server_;

//publisher
ros::Publisher state_publisher_;

hardware::states state_;

vector<geometry_msgs::Point> genStudPositions(string task_name)
{
    vector<geometry_msgs::Point> stud_positions;

    //here the real arm controller would check the distribution params on the param server
    // and "pose estimate" the wall section according to the task name.
    // From these, it would create the stud positions.
    // We just generate a set of evenly distributed positions within the size of the wall!

    //pseudo distribution params:
    double stud_distance = 0.1;
    double proximity = 0.15;

    double x;
    double y = proximity;

    while(y <= (WALL_HEIGHT - proximity)) // fill vertically
    {
        x = proximity;
        while(x <= (WALL_WIDTH - proximity)) // fill horizontally
        {
            //add stud:
            geometry_msgs::Point stud;
            stud.x = x;
            stud.y = y;
            stud.z = 0;
            stud_positions.push_back(stud);

            //update position
            x = x + stud_distance;
        }

        y = y + stud_distance;
    }

    return stud_positions;
}

void weldAllStuds(string task_name, int fail=-1)
{
    //get the stud list:
    XmlRpc::XmlRpcValue data;
    if(!ros::param::get(("mission/tasks/" + task_name + "/studs"), data))
    {
        ROS_WARN_STREAM("No studs in task: " << task_name);
        return;
    }

    //now process the studs:
    int i=0;
    for(XmlRpc::XmlRpcValue::iterator it = data.begin(); it != data.end(); it++)
    {
        //check if stud is pending:
        int stud_state;
        ros::param::get(("mission/tasks/" + task_name + "/studs/" + it->first + "/state"),stud_state);
        if(stud_state != (int)stud::PENDING)
            continue;                           //we don't increment i, because the "fail" is set according to "pending" studs

        mission_ctrl_msgs::executeWeldFeedback feedback;
        feedback.stud_id = it->first;

        feedback.stud_state = (i == fail ? false : true);
        weld_server_->publishFeedback(feedback);

        if(i == fail)
            return;
        i++;

        usleep(300000); //...otherwise feedback messages get lost!
    }
}

int selectFailStud(string task_name)
{
    //called if user has chosen "fail". The user must now choose which stud should fail:
    XmlRpc::XmlRpcValue data;
    if(!ros::param::get(("mission/tasks/" + task_name + "/studs"), data))
    {
        ROS_WARN_STREAM("No studs in task: " << task_name);
        return -1;
    }

    int pending_studs = 0;
    for(XmlRpc::XmlRpcValue::iterator it = data.begin(); it != data.end(); it++)
    {
        int stud_state;
        ros::param::get(("mission/tasks/" + task_name + "/studs/" + it->first + "/state"),stud_state);
        if(stud_state == (int)stud::PENDING)
            pending_studs++;
    }

    while(true)
    {
        int input;
        cout << "Task: " << task_name << " contains " << pending_studs << " pending studs. Which stud should fail? [1-" <<pending_studs<<"]" << endl;
        cout << endl;
        cout << "choice: ";
        cin >> input;

        if(input < 1 || input >pending_studs)
        {
            cout << "invalid choice, try again..." << endl << endl;
        }
        else
        {
            return input-1;
        }
    }

    return 0;
}

void weldActionCB(const mission_ctrl_msgs::executeWeldGoalConstPtr& goal)
{

    state_ = hardware::BUSY;

    mission_ctrl_msgs::executeWeldResult result;
    cout << BOLDWHITE << "-----------------------------------------" << endl;
    cout << BOLDGREEN << "Arm emulator - Welding" << endl;
    cout << BOLDWHITE << "-----------------------------------------" << RESET << endl << endl;

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
            weldAllStuds(goal->task_name);
                      result.result_state = true;
            weld_server_->setSucceeded(result);
            state_ = hardware::IDLE;
            return;
        }
        else if(input == "1")
        {
            int fail_stud = selectFailStud(goal->task_name);
            weldAllStuds(goal->task_name, fail_stud);
            result.result_state = false;
            result.error_string = "welding task failed";
            weld_server_->setAborted(result);
            state_ = hardware::IDLE;
            return;
        }
        else if(input == "2")
        {
            result.result_state = false;
            result.error_string = "hardware error";
            weld_server_->setAborted(result);
            state_ = hardware::ERROR;
            return;
        }
        else
        {
            cout << "invalid choice, try again..." << endl;
        }

    }
}

void genStudActionCB(const mission_ctrl_msgs::generateStudDistributionGoalConstPtr& goal)
{
    state_ = hardware::BUSY;

    mission_ctrl_msgs::generateStudDistributionResult result;

    ROS_DEBUG_STREAM("Received a generate stud distribution goal - task: " << goal->task_name);

    cout << BOLDWHITE << "-----------------------------------------" << endl;
    cout << BOLDGREEN << "Arm emulator - Generate Stud Positions" << endl;
    cout << BOLDWHITE << "-----------------------------------------" << RESET << endl << endl;

    if(!ros::param::has(("mission/tasks/" + goal->task_name)))
    {
        result.error_string = "task doesn't exist";
        gen_pos_server_->setAborted(result);
        state_ = hardware::IDLE;
        return;
    }

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
            result.positions = genStudPositions(goal->task_name);
            gen_pos_server_->setSucceeded(result);
            state_ = hardware::IDLE;
            return;
        }
        else if(input == "1")
        {
             result.error_string = "failed to generate stud positions";
            gen_pos_server_->setAborted(result);
            state_ = hardware::IDLE;
            return;
        }
        else if(input == "2")
        {
            result.error_string = "hardware error";
            gen_pos_server_->setAborted(result);
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

    weld_server_ = new actionlib::SimpleActionServer<mission_ctrl_msgs::executeWeldAction>(n, CARLOS_WELD_ACTION, boost::bind(&weldActionCB, _1), false);
    weld_server_->start();

    gen_pos_server_ = new actionlib::SimpleActionServer<mission_ctrl_msgs::generateStudDistributionAction>(n, CARLOS_DISTRIBUTION_ACTION, boost::bind(&genStudActionCB, _1), false);
    gen_pos_server_->start();

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
