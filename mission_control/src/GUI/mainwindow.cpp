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
#include "loadMission.hpp"
#include <iostream>
#include <QItemSelectionModel>
#include <QMessageBox>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "ros/service_client.h"
#include "mission_control/Trigger.h"
#include "mission_control/getMissionList.h"
#include "mission_control/getMissionData.h"
#include "mission_control/getTaskList.h"
#include "mission_control/getTaskData.h"
#include "mission_control/getTaskParams.h"
#include "mission_control/setTaskData.h"
#include "mission_control/setMissionData.h"
#include "mission_control/function_defines.h"
#include "mission_control/ui_api_defines.h"
#include "ros_interface.hpp"
#include "addTask.hpp"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle(tr("CARLOS Mission Controller"));

    initUI();

    //initialize ros rubscribers
    //    hw_state_sub = n.subscribe(UIAPI_HW_STATES, 10, &MainWindow::hwStateCB, this);
    //    exec_progress_sub = n.subscribe(UIAPI_EXEC_PROGRESS, 10, &MainWindow::execProgressCB, this);
    //    instr_progress_sub = n.subscribe(UIAPI_INSTR_PROGRESS, 10, &MainWindow::instrProgressCB, this);

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

    //qRegisterMetaType<mission_control::ProgressConstPtr>("ProgressPtr");

    //connect signal/slot for menubar
    connect(ui->closeButton,SIGNAL(clicked()),this,SLOT(close()));
    connect(ui->actionClose, SIGNAL(triggered()),this, SLOT(close()));
    connect(ui->actionNew, SIGNAL(triggered()),this, SLOT(createNew()));
    connect(ui->actionLoad, SIGNAL(triggered()),this, SLOT(load()));
    connect(ui->actionSave, SIGNAL(triggered()),this, SLOT(save()));
    connect(ui->actionSave_As, SIGNAL(triggered()),this, SLOT(saveAs()));
    //connect(this, SIGNAL(exec_progress_update_received(ProgressPtr)), this, SLOT(on_execProgressUpdate(ProgressPtr)));

    //ui->task_params_layout->addRow(QString("Tast"), new QLabel("hej med test"));

    //init tab widget:
    ui->tabWidget->setCurrentIndex(0); //always start at "info" tab

    //hide the edits:
    ui->mission_name_edit->hide();
    ui->cad_edit->hide();
    ui->mission_description_edit->hide();

    //remove the generate tasks button
    ui->gen_tasks_button->hide();

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
    ui->task_name_edit->hide();
    ui->stud_distribution_edit->hide();
    ui->stud_dist_edit->hide();
    ui->stud_proximity_edit->hide();
    ui->stud_type_edit->hide();
    ui->nav_x_edit->hide();
    ui->nav_y_edit->hide();
    ui->nav_yaw_edit->hide();
    ui->voltage_edit->hide();
    ui->direction_edit->hide();

    hideTaskParams();
}

void MainWindow::createNew()
{
    //check that a mission is loaded:
    mission_control::Trigger srv;
    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_GET_MISSION_NAME);
    if(!client.call(srv))
    {
        QMessageBox::warning(this,"Create new failed", "Couldn't get name of currently loaded mission",QMessageBox::Ok);
        return;
    }

    if(srv.response.message != "")
    {
        string loaded_name = srv.response.message;

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
    mission_control::getMissionList srv2;
    ros::ServiceClient client2 = n.serviceClient<mission_control::getMissionList>(UIAPI_GET_MISSION_LIST);
    if(!client2.call(srv2))
    {
        QMessageBox::warning(this,"Save failed","Couldn't get list of existing missions",QMessageBox::Ok);
        return;
    }

    vector<string> missions = srv2.response.missions;

    bool found = false;
    for(int i=0;i<(int)missions.size();i++)
    {
        if(name == missions[i])
        {
            found = true;
            break;
        }
    }

    if(found)
    {
        //prompt user for replace:
        int button = QMessageBox::question(this,"Replace",tr("Replace existing mission \"%1\"?").arg(QString::fromStdString(name)),
                                           QMessageBox::Yes | QMessageBox::Cancel);

        if (button == QMessageBox::Cancel)
            return;
    }

    //create the new mission
    mission_control::Trigger srv3;
    srv3.request.input = name;
    ros::ServiceClient client3 = n.serviceClient<mission_control::Trigger>(UIAPI_CREATE_NEW_MISSION);
    if(!client3.call(srv3))
    {
        QMessageBox::warning(this,"Create new failed","Failed to call create new service",QMessageBox::Ok);
        return;
    }

    updateInfo();
}

void MainWindow::load()
{
    //check if mission is currently open:
    mission_control::Trigger srv2;
    ros::ServiceClient client2 = n.serviceClient<mission_control::Trigger>(UIAPI_GET_MISSION_NAME);
    if(!client2.call(srv2))
    {
        QMessageBox::warning(this,"Load failed","Failed to get name of currently loaded mission",QMessageBox::Ok);
        return;
    }

    if(srv2.response.message != "")
    {
        string loaded_name = srv2.response.message;

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

    //load the selected mission
    mission_control::Trigger srv;
    srv.request.input = selected_mission;
    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_LOAD_MISSION);
    if(!client.call(srv))
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
    mission_control::Trigger srv;
    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_GET_MISSION_NAME);
    if(!client.call(srv))
    {
        QMessageBox::warning(this,"Save failed","Failed to call service",QMessageBox::Ok);
        return;
    }


    if(srv.response.message == "")
    {
        QMessageBox::warning(this,"Save failed","No mission loaded",QMessageBox::Ok);
        return;
    }

    //try to save:
    mission_control::Trigger srv2;
    ros::ServiceClient client2 = n.serviceClient<mission_control::Trigger>(UIAPI_SAVE_MISSION);
    if(!client2.call(srv2))
    {
        QMessageBox::warning(this,"Save failed","Failed to save mission",QMessageBox::Ok);
        return;
    }

    if(!srv2.response.success)    //must be because the mission doesn't have a name.
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
    mission_control::getMissionList srv;
    ros::ServiceClient client = n.serviceClient<mission_control::getMissionList>(UIAPI_GET_MISSION_LIST);
    if(!client.call(srv))
    {
        QMessageBox::warning(this,"Save failed","Calling service failed",QMessageBox::Ok);
        return;
    }

    vector<string> missions = srv.response.missions;

    bool found = false;
    for(int i=0;i<(int)missions.size();i++)
    {
        if(name == missions[i])
        {
            found = true;
            break;
        }
    }

    if(found)
    {
        //prompt user for replace:
        int button = QMessageBox::question(this,"Replace",tr("Replace existing mission \"%1\"?").arg(QString::fromStdString(name)),
                                           QMessageBox::Yes | QMessageBox::Cancel);

        if (button == QMessageBox::Cancel)
            return;
    }


    mission_control::Trigger srv2;
    srv2.request.input = name;
    ros::ServiceClient client2 = n.serviceClient<mission_control::Trigger>(UIAPI_SAVE_MISSION_AS);
    if(!client2.call(srv2))
    {
        QMessageBox::warning(this,"Save failed", "Calling service failed", QMessageBox::Ok);
        return;
    }

    if(!srv2.response.success)
    {
        QMessageBox::warning(this,"Save failed","Failed to save mission",QMessageBox::Ok);
        return;
    }
}

void MainWindow::on_taskList_clicked(const QModelIndex &index)
{
    //    string task = index.data().toString().toStdString();

    //    TaskParams params = mh_->getTaskData(task);

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

    updateTaskData(task);

    showTaskParams();
}

void MainWindow::updateTaskData(string task_name)
{
    mission_control::getTaskData srv;
    srv.request.name = task_name;
    ros::ServiceClient client = n.serviceClient<mission_control::getTaskData>(UIAPI_GET_TASK_DATA);
    if(!client.call(srv))
    {
        return;
    }

    ui->task_name_label->setText(QString::fromStdString(task_name));
    ui->stud_type_label->setText(QString::fromStdString(srv.response.data.stud_type));
    ui->stud_distribution_label->setText(QString::fromStdString(srv.response.data.stud_pattern.distribution));
    ui->stud_dist_label->setText(QString::number(srv.response.data.stud_pattern.distance));
    ui->stud_proximity_label->setText(QString::number(srv.response.data.stud_pattern.proximity));
    ui->nav_x_label->setText(QString::number(srv.response.data.nav_goal.x));
    ui->nav_y_label->setText(QString::number(srv.response.data.nav_goal.y));
    ui->nav_yaw_label->setText(QString::number(srv.response.data.nav_goal.yaw));
    ui->number_of_studs_label->setText(QString::number(srv.response.data.number_of_studs));
    ui->task_state->setText(QString::fromStdString(srv.response.data.state_description));
    ui->voltage_label->setText(QString::number(srv.response.data.voltage));
    ui->direction_label->setText(QString::number(srv.response.data.direction));
}

void MainWindow::updateTaskList()
{
    //get task list from mission handler:
    mission_control::getTaskList srv;
    ros::ServiceClient client = n.serviceClient<mission_control::getTaskList>(UIAPI_GET_TASK_LIST);
    if(!client.call(srv))
    {
        return;
    }

    vector<string> task_list = srv.response.tasks;

    //clear model:
    task_list_model->clear();

    for(int i=0;i<(int)task_list.size();i++)
    {
        task_list_model->appendRow(new QStandardItem(QString::fromStdString(task_list[i])));
    }

    //show/hide auto-gen button
    if((task_list.size() == 0) && (ui->mission_name->text().toStdString() != "<no mission>"))
        ui->gen_tasks_button->show();
    else
        ui->gen_tasks_button->hide();

    //hide/show task params
    if(ui->taskList->selectionModel()->selectedIndexes().size() < 1)
        hideTaskParams();
    else
        showTaskParams();
}

void MainWindow::updateMissionData()
{
    mission_control::Trigger srv;
    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_GET_MISSION_NAME);
    if(!client.call(srv))
    {
        return;
    }


    if(srv.response.message == "")
    {
        ui->mission_name->setText("<no mission>");
        ui->mission_description->setText("");
        ui->cad_ref->setText("<no CAD>");
        ui->mission_state->setText("");
        ui->last_saved->setText("");
        hideTaskParams();
        ui->editMissionButton->hide();
        ui->add_task_button->hide();
        return;
    }

    //a mission is loaded. Show the "edit" button
    ui->editMissionButton->show();

    //show the "add task" button
    ui->add_task_button->show();

    mission_control::getMissionData srv2;
    ros::ServiceClient client2 = n.serviceClient<mission_control::getMissionData>(UIAPI_GET_MISSION_DATA);
    if(!client2.call(srv2))
    {
        return;
    }

    ui->mission_name->setText(QString::fromStdString(srv2.response.data.name));
    ui->cad_ref->setText(QString::fromStdString(srv2.response.data.cad));
    ui->mission_state->setText(QString::fromStdString(srv2.response.data.state_description));
    ui->mission_description->setText(QString::fromStdString(srv2.response.data.description));

    //convert last saved double to a human readable formate:
    ros::Time mytime;
    mytime.fromSec(srv2.response.data.last_saved);
    boost::posix_time::ptime temp = mytime.toBoost();

    //reformat the output:
    boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%d-%b-%Y %H:%M:%S");
    stringstream ss;
    ss.imbue(std::locale(std::locale::classic(), facet));
    ss << temp << "  UTC";

    ui->last_saved->setText(QString::fromStdString(ss.str()));
}

void MainWindow::updateInfo()
{
    updateMissionData();
    updateTaskList();

}

void MainWindow::hideTaskParams()
{
    ui->task_name_label->hide();
    ui->stud_distribution_label->hide();
    ui->stud_dist_label->hide();
    ui->stud_proximity_label->hide();
    ui->stud_type_label->hide();
    ui->direction_label->hide();
    ui->voltage_label->hide();
    ui->nav_x_label->hide();
    ui->nav_y_label->hide();
    ui->nav_yaw_label->hide();
    ui->nav_x->hide();
    ui->nav_y->hide();
    ui->nav_yaw->hide();
    ui->number_of_studs_label->hide();
    ui->task_state->hide();
    ui->editTaskButton->hide();
    ui->delete_task_button->hide();
}

void MainWindow::showTaskParams()
{
    ui->task_name_label->show();
    ui->stud_distribution_label->show();
    ui->stud_dist_label->show();
    ui->stud_proximity_label->show();
    ui->stud_type_label->show();
    ui->direction_label->show();
    ui->voltage_label->show();
    ui->nav_x_label->show();
    ui->nav_y_label->show();
    ui->nav_yaw_label->show();
    ui->nav_x->show();
    ui->nav_y->show();
    ui->nav_yaw->show();
    ui->number_of_studs_label->show();
    ui->task_state->show();
    ui->editTaskButton->show();
    ui->delete_task_button->show();
}

void MainWindow::startMissionEdit()
{
    //set editButton icon and text:
    ui->editMissionButton->setText("Close");

    //disable tab-widget + task list
    ui->tabWidget->setEnabled(false);
    ui->taskList->setEnabled(false);

    //transfer data from labels to edits:
    ui->mission_name_edit->setText(ui->mission_name->text());
    ui->cad_edit->setText(ui->cad_ref->text());
    ui->mission_description_edit->setText(ui->mission_description->text());

    //hide the labels:
    ui->mission_name->hide();
    ui->cad_ref->hide();
    ui->mission_description->hide();

    //show the edits:
    ui->mission_name_edit->show();
    ui->cad_edit->show();
    ui->mission_description_edit->show();
}

void MainWindow::closeMissionEdit()
{
    //set editButton icon and text:
    ui->editMissionButton->setText("Edit");

    //enable tab-widget + task list
    ui->tabWidget->setEnabled(true);
    ui->taskList->setEnabled(true);

    //send the new data to MC
    mission_control::setMissionData srv;
    srv.request.name = ui->mission_name_edit->text().toStdString();
    srv.request.description = ui->mission_description_edit->text().toStdString();
    srv.request.cad = ui->cad_edit->text().toStdString();
    ros::ServiceClient client = n.serviceClient<mission_control::setMissionData>(UIAPI_SET_MISSION_DATA);
    if(!client.call(srv))
        return;

    if(!srv.response.success)
        return;

    //hide the edits
    ui->mission_name_edit->hide();
    ui->cad_edit->hide();
    ui->mission_description_edit->hide();

    //show the labels:
    ui->mission_name->show();
    ui->cad_ref->show();
    ui->mission_description->show();

    //update the data:
    updateMissionData();
}

void MainWindow::startTaskEdit()
{
    //set editButton icon and text:
    ui->editTaskButton->setText("Close");

    //disable other tabs + task list + mission edit button
    ui->tabWidget->setTabEnabled(1,false);
    ui->tabWidget->setTabEnabled(2,false);
    ui->taskList->setEnabled(false);
    ui->editMissionButton->setEnabled(false);

    //get the task params:
    mission_control::getTaskParams srv;
    ros::ServiceClient client = n.serviceClient<mission_control::getTaskParams>(UIAPI_GET_TASK_PARAMS);
    if(!client.call(srv))
    {
        return;
    }
    if(!srv.response.success)
    {
        return;
    }

    //set the data of the edits:
    ui->task_name_edit->setText(ui->task_name_label->text());

    ui->stud_dist_edit->setMaximum(srv.response.distance.max_value);
    ui->stud_dist_edit->setMinimum(srv.response.distance.min_value);
    ui->stud_dist_edit->setValue(ui->stud_dist_label->text().toDouble());
    if( srv.response.distance.max_value < ui->stud_dist_label->text().toDouble() || ui->stud_dist_label->text().toDouble() < srv.response.distance.min_value) //out of range!
        ui->stud_dist_edit->setValue(srv.response.distance.default_value);
    else
        ui->stud_dist_edit->setValue(ui->stud_dist_label->text().toDouble());

    ui->stud_proximity_edit->setMaximum(srv.response.proximity.max_value);
    ui->stud_proximity_edit->setMinimum(srv.response.proximity.min_value);
    ui->stud_proximity_edit->setValue(ui->stud_proximity_label->text().toDouble());
    if( srv.response.proximity.max_value < ui->stud_proximity_label->text().toDouble() || ui->stud_proximity_label->text().toDouble() < srv.response.proximity.min_value) //out of range!
        ui->stud_proximity_edit->setValue(srv.response.proximity.default_value);
    else
        ui->stud_proximity_edit->setValue(ui->stud_proximity_label->text().toDouble());

    ui->voltage_edit->setMaximum(srv.response.voltage.max_value);
    ui->voltage_edit->setMinimum(srv.response.voltage.min_value);
    if( srv.response.voltage.max_value < ui->voltage_label->text().toDouble() || ui->voltage_label->text().toDouble() < srv.response.voltage.min_value) //out of range!
        ui->voltage_edit->setValue(srv.response.voltage.default_value);
    else
        ui->voltage_edit->setValue(ui->voltage_label->text().toDouble());

    ui->direction_edit->setMaximum(srv.response.direction.max_value);
    ui->direction_edit->setMinimum(srv.response.direction.min_value);
    if( srv.response.direction.max_value < ui->direction_label->text().toInt() || ui->direction_label->text().toInt() < srv.response.direction.min_value) //out of range!
        ui->direction_edit->setValue(srv.response.direction.default_value);
    else
        ui->direction_edit->setValue(ui->direction_label->text().toInt());

    ui->nav_x_edit->setMinimum(-10000.0);
    ui->nav_x_edit->setMaximum(10000.0);
    ui->nav_x_edit->setValue(ui->nav_x_label->text().toDouble());

    ui->nav_y_edit->setMinimum(-10000.0);
    ui->nav_y_edit->setMaximum(10000.0);
    ui->nav_y_edit->setValue(ui->nav_y_label->text().toDouble());

    ui->nav_yaw_edit->setMinimum(-10000.0);
    ui->nav_yaw_edit->setMaximum(10000.0);
    ui->nav_yaw_edit->setValue(ui->nav_yaw_label->text().toDouble());

    ///Fill the distribution combo-box:
    int index = -1;
    int default_index = -1;
    ui->stud_distribution_edit->clear();
    for(int i=0;i<(int)srv.response.distributions.values.size();i++)
    {
        ui->stud_distribution_edit->addItem(QString::fromStdString(srv.response.distributions.values[i]));

        //check if the current value in the label matches any of the options! (it should do, otherwise, there is an error!)
        if(QString::fromStdString(srv.response.distributions.values[i]) == ui->stud_distribution_label->text())
            index = i;

        //find the default value's index
        if(srv.response.distributions.values[i] == srv.response.distributions.default_value)
            default_index = i;

    }
    //try to set the value to the current value:
    if(index == -1 && default_index != -1)
    {
        //use default value:
        ui->stud_distribution_edit->setCurrentIndex(default_index);
    }
    else if(index != -1)
    {
        ui->stud_distribution_edit->setCurrentIndex(index);
    }
    //else we dont do anything and the "default" will be what comes first. However, this should never happen.
    //if it does, there is an error in the data received from mission controller.

    ///Fill the stud type combobox
    index = -1;
    default_index = -1;
    ui->stud_type_edit->clear();
    for(int i=0;i<(int)srv.response.stud_types.values.size();i++)
    {
        ui->stud_type_edit->addItem(QString::fromStdString(srv.response.stud_types.values[i]));

        //check if the current value in the label matches any of the options! (it should do, otherwise, there is an error!)
        if(QString::fromStdString(srv.response.stud_types.values[i]) == ui->stud_type_label->text())
            index = i;

        //find the default value's index
        if(srv.response.stud_types.values[i] == srv.response.stud_types.default_value)
            default_index = i;

    }
    //try to set the value to the current value:
    if(index == -1 && default_index != -1)
    {
        //use default value:
        ui->stud_type_edit->setCurrentIndex(default_index);
    }
    else if(index != -1)
    {
        ui->stud_type_edit->setCurrentIndex(index);
    }

    //hide the qlabels
    ui->task_name_label->hide();
    ui->stud_distribution_label->hide();
    ui->stud_dist_label->hide();
    ui->stud_proximity_label->hide();
    ui->stud_type_label->hide();
    ui->nav_x_label->hide();
    ui->nav_y_label->hide();
    ui->nav_yaw_label->hide();
    ui->voltage_label->hide();
    ui->direction_label->hide();

    //show the edits
    ui->task_name_edit->show();
    ui->stud_distribution_edit->show();
    ui->stud_dist_edit->show();
    ui->stud_proximity_edit->show();
    ui->stud_type_edit->show();
    ui->nav_x_edit->show();
    ui->nav_y_edit->show();
    ui->nav_yaw_edit->show();
    ui->voltage_edit->show();
    ui->direction_edit->show();
}

void MainWindow::closeTaskEdit()
{
    //set editButton icon and text:
    ui->editTaskButton->setText("Edit");

    //enable other tabs + task list + mission edit
    ui->tabWidget->setTabEnabled(1,true);
    ui->tabWidget->setTabEnabled(2,true);
    ui->taskList->setEnabled(true);
    ui->editMissionButton->setEnabled(true);

    //hide the edits
    ui->task_name_edit->hide();
    ui->stud_distribution_edit->hide();
    ui->stud_dist_edit->hide();
    ui->stud_proximity_edit->hide();
    ui->stud_type_edit->hide();
    ui->nav_x_edit->hide();
    ui->nav_y_edit->hide();
    ui->nav_yaw_edit->hide();
    ui->voltage_edit->hide();
    ui->direction_edit->hide();

    //Send data to MC
    mission_control::setTaskData srv;
    srv.request.name = ui->task_name_edit->text().toStdString();
    srv.request.data.stud_type = ui->stud_type_edit->currentText().toStdString();
    srv.request.data.stud_pattern.distance = ui->stud_dist_edit->value();
    srv.request.data.stud_pattern.proximity = ui->stud_proximity_edit->value();
    srv.request.data.stud_pattern.distribution = ui->stud_distribution_edit->currentText().toStdString();
    srv.request.data.nav_goal.x = ui->nav_x_edit->value();
    srv.request.data.nav_goal.y = ui->nav_y_edit->value();
    srv.request.data.nav_goal.yaw = ui->nav_yaw_edit->value();
    srv.request.data.direction = ui->direction_edit->value();
    srv.request.data.voltage = ui->voltage_edit->value();


    ros::ServiceClient client = n.serviceClient<mission_control::setTaskData>(UIAPI_SET_TASK_DATA);
    if(!client.call(srv))
        return;

    if(!srv.response.success)
        return;

    //update the task list:
    updateTaskList();

    //set the task list current index to the current task
    //find the "index" of the current task:
    for(int i=0;i<task_list_model->rowCount();i++)
    {
        if(task_list_model->index(i,0).data(Qt::DisplayRole).toString() == ui->task_name_edit->text())
        {
            ui->taskList->selectionModel()->select(task_list_model->index(i,0), QItemSelectionModel::Select);
        }
    }

    //show the task data labels
    showTaskParams();

    //update the task data
    updateTaskData(ui->task_name_edit->text().toStdString());
    updateMissionData();
}

void MainWindow::on_editMissionButton_clicked()
{
    if(ui->mission_name->text().toStdString() == "<no mission>")
    {
        //no mission loaded! This shouldn't happen, as the user should not be able to click the "edit" button.
        return;
    }

    //check if edit begin or end:
    if(ui->editMissionButton->text().toStdString() == "Edit")
    {
        //go to edit mode:
        mission_control::Trigger srv;
        ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EDIT_START);
        if(!client.call(srv))
            return;

        if(!srv.response.success)
            return;

        startMissionEdit();
    }
    else
    {
        closeMissionEdit();

        mission_control::Trigger srv;
        ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EDIT_STOP);
        if(!client.call(srv))
            return;

        if(!srv.response.success)
            return;
    }
}

void MainWindow::on_editTaskButton_clicked()
{
    if(ui->task_name_label->isHidden() && ui->task_name_edit->isHidden())
    {
        //no task selected! This shouldn't happen, as the user should not be able to click the "edit" button.
        return;
    }

    //check if edit begin or end:
    if(ui->editTaskButton->text() == "Edit")
    {
        //go to edit mode:
        mission_control::Trigger srv;
        ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EDIT_START);
        if(!client.call(srv))
            return;

        if(!srv.response.success)
            return;

        startTaskEdit();
    }
    else
    {
        closeTaskEdit();

        mission_control::Trigger srv;
        ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EDIT_STOP);
        if(!client.call(srv))
            return;

        if(!srv.response.success)
            return;
    }
}

void MainWindow::on_execStartButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EXEC_START);
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
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_INSTR_START);
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

void MainWindow::hwStateUpdate(mission_control::HardwareStates msg)
{
    //update state labels in bottom of window
    ui->robot_state_label->setText(QString::fromStdString(msg.hardware_states[0].device_state));
    ui->platform_state_label->setText(QString::fromStdString(msg.hardware_states[1].device_state));
    ui->arm_state_label->setText(QString::fromStdString(msg.hardware_states[2].device_state));
    ui->system_state_label->setText(QString::fromStdString(msg.hardware_states[3].device_state));
}

void MainWindow::execProgressUpdate(const mission_control::Progress::ConstPtr &msg)
{
    //update the meta info:
    updateInfo();

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

    //kill:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == EXEC_KILL)
        {
            if(msg->enabled_functions[i].enabled)
                ui->execKillButton->show();
            else
                ui->execKillButton->hide();
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
}

void MainWindow::instrProgressUpdate(const mission_control::Progress::ConstPtr &msg)
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

    //kill:
    for(int i=0;i<(int)msg->enabled_functions.size();i++)
    {
        if(msg->enabled_functions[i].name == INSTR_KILL)
        {
            if(msg->enabled_functions[i].enabled)
                ui->instrKillButton->show();
            else
                ui->instrKillButton->hide();
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
    updateMissionData();
}

void MainWindow::on_add_task_button_clicked()
{
    //go to edit mode:
    mission_control::Trigger srv;
    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EDIT_START);
    if(!client.call(srv))
        return;

    if(!srv.response.success)
        return;

    //open add task dialog
    AddTask dialog(this);
    dialog.exec();


    //exit edit mode:
    mission_control::Trigger srv2;
    ros::ServiceClient client2 = n.serviceClient<mission_control::Trigger>(UIAPI_EDIT_STOP);
    if(!client2.call(srv2))
        return;

    if(!srv2.response.success)
        return;

    //update the task list:
    updateTaskList();
}

void MainWindow::on_delete_task_button_clicked()
{
    //get name of selected task:
    string task_name = ui->task_name_label->text().toStdString();

    //check "are you sure"
    int button = QMessageBox::question(this,"Delete task",tr("Are you sure you want to delete \"%1\"?").arg(QString::fromStdString(task_name)),
                                       QMessageBox::No | QMessageBox::Yes);

    if (button == QMessageBox::No)
        return;

    //go to edit mode:
    enterEditMode();

    //send delete task request:
    mission_control::Trigger srv2;
    srv2.request.input = task_name;
    ros::ServiceClient client2 = n.serviceClient<mission_control::Trigger>(UIAPI_DELETE_TASK);
    if(!client2.call(srv2))
    {
        QMessageBox::warning(this,"Error","Failed to delete task.",QMessageBox::Ok);
    }

    if(!srv2.response.success)
    {
        QMessageBox::warning(this,"Error","Failed to delete task.",QMessageBox::Ok);
    }

    //exit edit mode
    exitEditMode();

    //update the task list:
    updateTaskList();
}

bool MainWindow::enterEditMode()
{
    mission_control::Trigger srv;
    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EDIT_START);
    if(!client.call(srv))
        return false;

    if(!srv.response.success)
        return false;

    return true;
}

bool MainWindow::exitEditMode()
{
    mission_control::Trigger srv;
    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EDIT_STOP);
    if(!client.call(srv))
        return false;

    if(!srv.response.success)
        return false;

    return true;
}

void MainWindow::on_gen_tasks_button_clicked()
{
    //check "are you sure"
    //prompt user for verification:
    int button = QMessageBox::question(this,"Auto-generate tasks",tr("This will auto-generate a number of tasks. Are you sure?"),
                                       QMessageBox::No | QMessageBox::Yes);

    if (button == QMessageBox::No)
        return;

    //go to edit mode:
    enterEditMode();

    //send delete task request:
    mission_control::Trigger srv2;
    ros::ServiceClient client2 = n.serviceClient<mission_control::Trigger>(UIAPI_GEN_TASKS);
    if(!client2.call(srv2))
    {
        QMessageBox::warning(this,"Error","Failed to call service for auto-generate tasks.",QMessageBox::Ok);
    }

    if(!srv2.response.success)
    {
        QMessageBox::warning(this,"Error","Failed to auto-generate tasks.",QMessageBox::Ok);
    }

    //exit edit mode
    exitEditMode();

    //update mission info and task list:
    usleep(2000000);
    updateInfo();

}

void MainWindow::on_execKillButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_EXEC_KILL);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_instrKillButton_clicked()
{
    mission_control::Trigger srv;

    ros::ServiceClient client = n.serviceClient<mission_control::Trigger>(UIAPI_INSTR_KILL);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }
}

void MainWindow::on_clearStudsButton_clicked()
{
    //go to edit mode
}
