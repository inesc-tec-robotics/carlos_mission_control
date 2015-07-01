/***************************************************************************
 * Software License Agreement (BSD License)                                *
 *                                                                         *
 *  Copyright (c) 2015, Casper Schou                                       *
 *	Department of Mechanical and Manufacturing Engineering                 *
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
 * This is the first window in the GUI. The grapical layout is created in the ui-header file
 * "ui_MainWindow.h".
 *
 *
 */

#include "mainwindow.h"
#include "../mission_handler.hpp"
#include "loadMission.hpp"
#include <iostream>
#include <QItemSelectionModel>
#include "ros/service_client.h"
#include "mission_control/Start.h"
#include "mission_control/Trigger.h"
#include "mission_control/function_defines.h"
#include "mission_control/ui_api_defines.h"


using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    mh_ = MissionHandler::getInstance();

    ui->setupUi(this);
    setWindowTitle(tr("CARLOS Mission Controller"));

    initUI();

    //initialize ros rubscribers
    hw_state_sub = n.subscribe(UIAPI_HW_STATES, 10, &MainWindow::hwStateCB, this);
    exec_progress_sub = n.subscribe(UIAPI_EXEC_PROGRESS, 10, &MainWindow::execProgressCB, this);
    instr_progress_sub = n.subscribe(UIAPI_INSTR_PROGRESS, 10, &MainWindow::instrProgressCB, this);

    updateInfo();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initUI()
{
    task_list_model = new QStandardItemModel();
    ui->taskList->setModel(task_list_model);
    connect(ui->taskList->selectionModel(),
            SIGNAL(currentChanged(QModelIndex,QModelIndex)),
            this,
            SLOT(on_taskList_selection_changed(QModelIndex, QModelIndex)) );


    //connect signal/slot for menubar
    connect(ui->closeButton,SIGNAL(clicked()),this,SLOT(close()));
    connect(ui->actionClose, SIGNAL(triggered()),this, SLOT(close()));
    connect(ui->actionNew, SIGNAL(triggered()),this, SLOT(createNew()));
    connect(ui->actionLoad, SIGNAL(triggered()),this, SLOT(load()));
    connect(ui->actionSave, SIGNAL(triggered()),this, SLOT(save()));
    connect(ui->actionSave_As, SIGNAL(triggered()),this, SLOT(saveAs()));

    //ui->task_params_layout->addRow(QString("Tast"), new QLabel("hej med test"));

    //init tab widget:
    ui->tabWidget->setCurrentIndex(0); //always start at "info" tab

    //init the tabs:
    initInfoTab();
    initExecuteTab();
    initInstructTab();

}

void MainWindow::initExecuteTab()
{
    ui->exec_status_label->hide();
    ui->exec_descrp_label->hide();
}

void MainWindow::initInstructTab()
{
    ui->instr_status_label->hide();
    ui->instr_descrp_label->hide();
}

void MainWindow::initInfoTab()
{
    hideTaskParams();
}

void MainWindow::createNew()
{
    //check if mission is currently open:
    if(mh_->isLoaded())
    {
        string loaded_name = mh_->getLoadedName();

        if(loaded_name == "")   //hence, not given a name yet
            loaded_name = "<not saved>";

        //prompt user for save:
        int button = QMessageBox::question(this,"Save current mission",tr("Save mission \"%1\"?").arg(QString::fromStdString(loaded_name)),
                                           QMessageBox::Close | QMessageBox::Save | QMessageBox::Cancel); // Message popup to confirm remove of skill

        if (button == QMessageBox::Cancel)
            return;
        else if (button == QMessageBox::Save)
            save();
    }

    //Prompt user to enter name:
    QString user_input = QInputDialog::getText(this, "Enter mission name", "Name");

    //Did user input anything?
    if(user_input.isEmpty())
    {
        QMessageBox::warning(this,tr("Missing input"),tr("Please enter a name"),QMessageBox::Ok);
        return;
    }

    string name = user_input.toStdString();

    //check name for any potential ending:
    size_t ext_found = name.find(".");
    if(ext_found != string::npos)
    {
        name.erase(ext_found);
    }

    //check if the name entered exists!
    if(mh_->isMission(name))
    {
        //prompt user for replace:
        int button = QMessageBox::question(this,"Replace",tr("Replace existing mission \"%1\"?").arg(QString::fromStdString(name)),
                                           QMessageBox::Yes | QMessageBox::Cancel);

        if (button == QMessageBox::Cancel)
            return;
    }

    //create the new mission
    mh_->createNew(name);

    //save the new mission
    mh_->save();
}

void MainWindow::load()
{
    //check if mission is currently open:
    if(mh_->isLoaded())
    {
        string loaded_name = mh_->getLoadedName();

        if(loaded_name == "")   //hence, not given a name yet
            loaded_name = "<not saved>";

        //prompt user for save:
        int button = QMessageBox::question(this,"Save current mission",tr("Save mission \"%1\"?").arg(QString::fromStdString(loaded_name)),
                                           QMessageBox::Close | QMessageBox::Save | QMessageBox::Cancel); // Message popup to confirm remove of skill

        if (button == QMessageBox::Cancel)
            return;
        else if (button == QMessageBox::Save)
            save();
    }

    //open load mission dialog
    LoadMission dialog(this);
    if (!dialog.exec())
        return;
    string selected_mission = dialog.getSelectedMission();

    //open the selected mission
    if(!mh_->load(selected_mission))
    {
        QMessageBox::warning(this,"Error","Failed to load mission. Does it exist?",QMessageBox::Ok);
        return;
    }

    //fill the task list;
    updateInfo();

}

void MainWindow::save()
{
    //check that a mission is loaded:
    if(!mh_->isLoaded())
    {
        QMessageBox::warning(this,"Save failed","No mission loaded",QMessageBox::Ok);
        return;
    }

    if(!mh_->save())    //must be because the mission doesn't have a name.
    {
        saveAs();
    }
}

void MainWindow::saveAs()
{
    //Prompt user to enter name:
    QString user_input = QInputDialog::getText(this, "Enter mission name", "Name");

    //Did user input anything?
    if(user_input.isEmpty())
    {
        QMessageBox::warning(this,tr("Missing input"),tr("Please enter a name"),QMessageBox::Ok);
        return;
    }

    string name = user_input.toStdString();

    //check name for any potential ending:
    size_t ext_found = name.find(".");
    if(ext_found != string::npos)
    {
        name.erase(ext_found);
    }

    //check if the name entered exists!
    if(mh_->isMission(name))
    {
        //prompt user for replace:
        int button = QMessageBox::question(this,"Replace",tr("Replace existing mission \"%1\"?").arg(QString::fromStdString(name)),
                                           QMessageBox::Yes | QMessageBox::Cancel);

        if (button == QMessageBox::Cancel)
            return;
    }

    if(!mh_->saveAs(name))
    {
        QMessageBox::warning(this,"Save failed","Failed to save mission",QMessageBox::Ok);
        return;
    }
}

void MainWindow::on_taskList_clicked(const QModelIndex &index)
{
    //    string task = index.data().toString().toStdString();

    //    TaskParams params = mh_->getTaskParams(task);

    //    ui->task_name_label->setText(QString::fromStdString(params.name));
    //    ui->stud_type_label->setText(QString::fromStdString(params.stud_type));
    //    ui->stud_distribution_label->setText(QString::fromStdString(params.stud_pattern.distribution));
    //    ui->stud_dist_label->setText(QString::number(params.stud_pattern.distance));
    //    ui->stud_proximity_label->setText(QString::number(params.stud_pattern.proximity));
    //    ui->nav_goal_label->setText(QString::fromStdString( params.navigation_goal.toString(true) ));

}

void MainWindow::on_taskList_selection_changed(const QModelIndex &index, const QModelIndex &deselected)
{
    string task = index.data().toString().toStdString();

    TaskParams params = mh_->getTaskParams(task);

    ui->task_name_label->setText(QString::fromStdString(params.name));
    ui->stud_type_label->setText(QString::fromStdString(params.stud_type));
    ui->stud_distribution_label->setText(QString::fromStdString(params.stud_pattern.distribution));
    ui->stud_dist_label->setText(QString::number(params.stud_pattern.distance));
    ui->stud_proximity_label->setText(QString::number(params.stud_pattern.proximity));
    ui->nav_goal_label->setText(QString::fromStdString( params.navigation_goal.toString(true) ));
    ui->number_of_studs_label->setText(QString::number(params.studs.size()));
    ui->task_state->setText(QString::fromStdString(params.state.toString()));

    showTaskParams();
}

void MainWindow::updateTaskList()
{
    //get task list from mission handler:
    vector<string> task_list = mh_->getTaskList();

    //clear model:
    task_list_model->clear();

    for(int i=0;i<(int)task_list.size();i++)
    {
        task_list_model->appendRow(new QStandardItem(QString::fromStdString(task_list[i])));
    }

    //    if(task_list.size() == 0)
    //        hideTaskParams();
    //    else
    //        showTaskParams();

    if(ui->taskList->selectionModel()->selectedIndexes().size() < 1)
        hideTaskParams();
    else
        showTaskParams();
}

void MainWindow::updateMissionMeta()
{
    if(!mh_->isLoaded())
    {
        ui->mission_name->setText("<no mission>");
        ui->cad_ref->setText("<no CAD>");
        ui->mission_state->setText("");
        hideTaskParams();
        return;
    }

    MissionParams params = mh_->getMissionParams();
    ui->mission_name->setText(QString::fromStdString(params.name));
    ui->cad_ref->setText(QString::fromStdString(params.CAD_ref));
    ui->mission_state->setText(QString::fromStdString(params.state.toString()));
}

void MainWindow::updateInfo()
{
    updateMissionMeta();
    if(mh_->isLoaded())
        updateTaskList();

}

void MainWindow::hideTaskParams()
{
    ui->task_name_label->hide();
    ui->stud_distribution_label->hide();
    ui->stud_dist_label->hide();
    ui->stud_proximity_label->hide();
    ui->stud_type_label->hide();
    ui->nav_goal_label->hide();
    ui->number_of_studs_label->hide();
    ui->task_state->hide();
}

void MainWindow::showTaskParams()
{
    ui->task_name_label->show();
    ui->stud_distribution_label->show();
    ui->stud_dist_label->show();
    ui->stud_proximity_label->show();
    ui->stud_type_label->show();
    ui->nav_goal_label->show();
    ui->number_of_studs_label->show();
    ui->task_state->show();
}

void MainWindow::hwStateCB(const mission_control::HardwareStates::ConstPtr &msg)
{
    //update state labels in bottom of window
    ui->robot_state_label->setText(QString::fromStdString(msg->hardware_states[0].device_state));
    ui->platform_state_label->setText(QString::fromStdString(msg->hardware_states[1].device_state));
    ui->arm_state_label->setText(QString::fromStdString(msg->hardware_states[2].device_state));
    ui->system_state_label->setText(QString::fromStdString(msg->hardware_states[3].device_state));
}

void MainWindow::execProgressCB(const mission_control::Progress::ConstPtr &msg)
{
    //unhide the execution status and description labels:
    ui->exec_status_label->show();
    ui->exec_descrp_label->show();

    //update the exection status and description:
    ui->exec_status_label->setText(QString::fromStdString(msg->engine_state));
    ui->exec_descrp_label->setText(QString::fromStdString(msg->description));
    ui->mission_name->setText(QString::fromStdString(msg->current_mission));

    //update the task list (color the current task)
    for(int i=0;i<task_list_model->rowCount();i++)
    {
        if(task_list_model->index(i,0).data(Qt::DisplayRole).toString().toStdString() == msg->current_task)
            task_list_model->setData(task_list_model->index(i,0),Qt::red,Qt::ForegroundRole);
        else
            task_list_model->setData(task_list_model->index(i,0),Qt::black,Qt::ForegroundRole);
    }

    //update the enabled buttons:
    //start:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == EXEC_START)
        {
            ui->execStartButton->setEnabled(msg->enabled_functions[i].enabled);
            break;
        }
    }

    //pause:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == EXEC_PAUSE)
        {
            ui->execPauseButton->setEnabled(msg->enabled_functions[i].enabled);
            break;
        }
    }

    //abort:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == EXEC_ABORT)
        {
            ui->execStopButton->setEnabled(msg->enabled_functions[i].enabled);
            break;
        }
    }

    //retry:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == EXEC_RETRY)
        {
            ui->execRetryButton->setEnabled(msg->enabled_functions[i].enabled);
            break;
        }
    }

    //skipStud:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == EXEC_SKIP_STUD)
        {
            ui->execSkipStudButton->setEnabled(msg->enabled_functions[i].enabled);
        }
    }

    //skipTask:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == EXEC_SKIP_TASK)
        {
            ui->execSkipTaskButton->setEnabled(msg->enabled_functions[i].enabled);
        }
    }

    //update the meta info:
    updateInfo();
}

void MainWindow::instrProgressCB(const mission_control::Progress::ConstPtr &msg)
{
    //unhide the execution status and description labels:
    ui->instr_status_label->show();
    ui->instr_descrp_label->show();

    //update the exection status and description:
    ui->instr_status_label->setText(QString::fromStdString(msg->engine_state));
    ui->instr_descrp_label->setText(QString::fromStdString(msg->description));
    ui->mission_name->setText(QString::fromStdString(msg->current_mission));

    //update the task list (color the current task)
    for(int i=0;i<task_list_model->rowCount();i++)
    {
        if(task_list_model->index(i,0).data(Qt::DisplayRole).toString().toStdString() == msg->current_task)
            task_list_model->setData(task_list_model->index(i,0),Qt::red,Qt::ForegroundRole);
        else
            task_list_model->setData(task_list_model->index(i,0),Qt::black,Qt::ForegroundRole);
    }

    //update the enabled buttons:
    //start:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == INSTR_START)
        {
            ui->instrStartButton->setEnabled(msg->enabled_functions[i].enabled);
            break;
        }
    }

    //pause:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == INSTR_PAUSE)
        {
            ui->instrPauseButton->setEnabled(msg->enabled_functions[i].enabled);
            break;
        }
    }

    //abort:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == INSTR_ABORT)
        {
            ui->instrStopButton->setEnabled(msg->enabled_functions[i].enabled);
            break;
        }
    }

    //retry:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == INSTR_RETRY)
        {
            ui->instrRetryButton->setEnabled(msg->enabled_functions[i].enabled);
            break;
        }
    }

    //skipTask:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == INSTR_SKIP_TASK)
        {
            ui->instrSkipTaskButton->setEnabled(msg->enabled_functions[i].enabled);
        }
    }

    //update the meta info:
    updateInfo();
}


void MainWindow::on_execStartButton_clicked()
{
    mission_control::Start srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Start>(UIAPI_EXEC_START);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_execPauseButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EXEC_PAUSE_RESUME);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_execStopButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EXEC_ABORT);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_execRetryButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EXEC_RETRY);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_execSkipTaskButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EXEC_SKIP_TASK);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_execSkipStudButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EXEC_SKIP_STUD);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}


void MainWindow::on_instrStartButton_clicked()
{
    mission_control::Start srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Start>(UIAPI_INSTR_START);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_instrPauseButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_INSTR_PAUSE_RESUME);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_instrStopButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_INSTR_ABORT);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_instrRetryButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_INSTR_RETRY);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_instrSkipTaskButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_INSTR_SKIP_TASK);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}
