#ifndef ROSINTERFACE_HPP_
#define ROSINTERFACE_HPP_

#include <QMainWindow>
#include <QDebug>
#include <QtGui>
#include <QIcon>
#include <QWidget>
#include "ui_mainwindow.h"
#include "ros/node_handle.h"
#include "mission_control/HardwareStates.h"
#include "mission_control/Progress.h"

class RosInterface : public QThread
{
    Q_OBJECT

//public:
//    RosInterface();
//    ~RosInterface();

signals:
    void hw_states_received(mission_control::HardwareStates msg);
    void exec_progress_update_received(const mission_control::Progress::ConstPtr &msg);
    void instr_progress_update_received(const mission_control::Progress::ConstPtr &msg);

private:

    void run();

    void hwStateCB(const mission_control::HardwareStates::ConstPtr &msg);
    void execProgressCB(const mission_control::Progress::ConstPtr &msg);
    void instrProgressCB(const mission_control::Progress::ConstPtr &msg);

    ros::NodeHandle n;
    ros::Subscriber hw_state_sub;
    ros::Subscriber exec_progress_sub;
    ros::Subscriber instr_progress_sub;




};

#endif // ROSINTERFACE_HPP_
