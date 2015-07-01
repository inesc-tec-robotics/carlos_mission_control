
//
//  main.cpp
//
//  Created by Casper Schou @ AAU on 14/04/2015
//  Copyright (c) 2015 Casper Schou. All rights reserved.
//
#include <QtCore>
//#include "ros/ros.h"
#include "GUI/mainwindow.h"
#include "GUI/ros_interface.hpp"

using namespace std;

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "mission_controller_gui");

    QApplication a(argc, argv);
    setlocale(LC_NUMERIC,"C");
    MainWindow w;

    RosInterface ros_interface;


    qRegisterMetaType<mission_control::Progress::ConstPtr>("mission_control::Progress::ConstPtr");
    qRegisterMetaType<mission_control::HardwareStates>("mission_control::HardwareStates");
    QObject::connect(&ros_interface, SIGNAL(exec_progress_update_received(const mission_control::Progress::ConstPtr&)), &w, SLOT(execProgressUpdate(const mission_control::Progress::ConstPtr&)));
    QObject::connect(&ros_interface, SIGNAL(instr_progress_update_received(const mission_control::Progress::ConstPtr&)), &w, SLOT(instrProgressUpdate(const mission_control::Progress::ConstPtr&)));
    QObject::connect(&ros_interface, SIGNAL(hw_states_received(mission_control::HardwareStates)), &w, SLOT(hwStateUpdate(mission_control::HardwareStates)));


    ros_interface.start();
    w.show();

    //    ros::AsyncSpinner spinner(1);
    //    spinner.start();
    a.exec();

    return 0;
}
