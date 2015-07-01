 
//
//  main.cpp
//
//  Created by Casper Schou @ AAU on 14/04/2015
//  Copyright (c) 2015 Casper Schou. All rights reserved.
//

#include "ros/ros.h"
#include "ros/node_handle.h"
#include "UI_API.hpp"
#include "GUI/GUImain.h"
#include "hw_state_machine.hpp"
#include "system_engine.hpp"
#include "execution_engine.hpp"
#include "instruction_engine.hpp"


using namespace std;

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "mission_control");

    ros::NodeHandle n;

    //Start hw_state_machine
    HardwareStateMachine::getInstance();

    //start system state machine
    SystemEngine::getInstance();

    //start execution engine
    ExecutionEngine::getInstance();

    //start instruction engine
    InstructionEngine::getInstance();

    //launch UI api:
    ROS_INFO("Launching UI API...");
    UiAPI::getInstance();


//    GUImain* myGUI;

//    //Start GUI if requested
//    if(argc >= 2)
//    {
//        if(!strcmp(argv[1], "GUI") || !strcmp(argv[1], "gui") || !strcmp(argv[1], "Gui"))
//        {
//            ROS_INFO("Launching native GUI...");
//            myGUI = new GUImain(argc, argv);
//        }
//    }

    ROS_INFO("mission_controller running...");

    ros::spin();
    ros::waitForShutdown();

    return 0;
}
