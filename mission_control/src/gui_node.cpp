 
//
//  main.cpp
//
//  Created by Casper Schou @ AAU on 14/04/2015
//  Copyright (c) 2015 Casper Schou. All rights reserved.
//
#include <QtCore>
#include "ros/ros.h"
#include "GUI/mainwindow.h"

using namespace std;

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "mission_controller_gui");

    QApplication a(argc, argv);
    setlocale(LC_NUMERIC,"C");
    MainWindow w;
    w.show();

    ros::AsyncSpinner spinner(1);
    spinner.start();
    a.exec();

    return 0;
}
