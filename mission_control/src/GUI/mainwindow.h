/***************************************************************************
 * Software License Agreement (BSD License)                                *
 *                                                                         *
 *  Copyright (c) 2012, Mikkel Hvilshoj, Christian Caroe, Casper Schou     *
 *	Department of Mechanical and Manufacturing Engineering             *
 *  Aalborg University, Denmark                                            *
 *  All rights reserved.                                                   *
 *                                                                         *
 *  Redistribution and use in source and binary forms, with or without     *
 *  modification, are permitted provided that the following conditions     *
 *  are met:                                                               *
 *                                                                         *
 *  - Redistributions of source code must retain the above copyright       *
 *     notice, this list of conditions and the following disclaimer.       *
 *  - Redistributions in binary form must reproduce the above              *
 *     copyright notice, this list of conditions and the following         *
 *     disclaimer in the documentation and/or other materials provided     *
 *     with the distribution.                                              *
 *  - Neither the name of Aalborg University nor the names of              *
 *     its contributors may be used to endorse or promote products derived *
 *     from this software without specific prior written permission.       *
 *                                                                         *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    *
 *  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS      *
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE         *
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    *
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,   *
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;       *
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT     *
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN      *
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        *
 *  POSSIBILITY OF SUCH DAMAGE.                                            *
 ***************************************************************************
 *
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QtGui>
#include <QIcon>
#include <QWidget>
#include "ui_mainwindow.h"
#include "ros/node_handle.h"
#include "mission_control/HardwareStates.h"
#include "mission_control/Progress.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void createNew();

    void load();

    void save();
    void saveAs();


    void on_taskList_clicked(const QModelIndex &index);

    void on_taskList_selection_changed(const QModelIndex & index, const QModelIndex & deselected);

    void on_execStartButton_clicked();

    void on_execPauseButton_clicked();

    void on_execStopButton_clicked();

    void on_execRetryButton_clicked();

    void on_execSkipTaskButton_clicked();

    void on_execSkipStudButton_clicked();

    void on_instrStartButton_clicked();

    void on_instrPauseButton_clicked();

    void on_instrStopButton_clicked();

    void on_instrRetryButton_clicked();

    void on_instrSkipTaskButton_clicked();

    void on_editTaskButton_clicked();

    void on_editMissionButton_clicked();

    void on_add_task_button_clicked();

    void on_delete_task_button_clicked();

public slots:
    void execProgressUpdate(const mission_control::Progress::ConstPtr &msg);
    void instrProgressUpdate(const mission_control::Progress::ConstPtr &msg);
    void hwStateUpdate(mission_control::HardwareStates msg);

signals:

    //void exec_progress_update_received(mission_control::ProgressConstPtr);

private:

    void initUI();
    void initExecuteTab();
    void initInstructTab();
    void initInfoTab();

    void updateTaskList();

    void updateMissionData();
    void updateTaskData(std::string task_name);

    void updateInfo();

    void hideTaskParams();
    void showTaskParams();

    //start/stop editing mission
    void startMissionEdit();
    void closeMissionEdit();

    //start/stop editing of task
    void startTaskEdit();
    void closeTaskEdit();

    bool enterEditMode();
    bool exitEditMode();

    //    void hwStateCB(const mission_control::HardwareStates::ConstPtr &msg);
    //    void execProgressCB(const mission_control::Progress::ConstPtr &msg);
    //    void instrProgressCB(const mission_control::Progress::ConstPtr &msg);

    Ui::MainWindow *ui;

    QStandardItemModel* task_list_model;

    ros::NodeHandle n;
    ros::Subscriber hw_state_sub;
    ros::Subscriber exec_progress_sub;
    ros::Subscriber instr_progress_sub;




};

#endif // MAINWINDOW_H
