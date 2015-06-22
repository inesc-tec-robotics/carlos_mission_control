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


using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    mh_ = MissionHandler::getInstance();

    ui->setupUi(this);
    setWindowTitle(tr("CARLOS Mission Controller"));

    //setWindowIcon(QIcon(QString::fromStdString(pic)));
    //QWidget::showMaximized();

    initUI();

    //TIMER: Could be used to update data from param:
    //    QTimer *timer = new QTimer(this);
    //    timer->setInterval(1000);
    //    connect(timer, SIGNAL(timeout()), this, SLOT(updateDeviceList()));
    //    timer->start();

    hideTaskParams();
    update();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initUI()
{
    task_list_model = new QStandardItemModel();
    ui->taskList->setModel(task_list_model);

    //connect signal/slot for menubar
    connect(ui->closeButton,SIGNAL(clicked()),this,SLOT(close()));
    connect(ui->actionClose, SIGNAL(triggered()),this, SLOT(close()));
    connect(ui->actionNew, SIGNAL(triggered()),this, SLOT(createNew()));
    connect(ui->actionLoad, SIGNAL(triggered()),this, SLOT(load()));
    connect(ui->actionSave, SIGNAL(triggered()),this, SLOT(save()));
    connect(ui->actionSave_As, SIGNAL(triggered()),this, SLOT(saveAs()));

    connect(ui->taskList->selectionModel(),
            SIGNAL(currentChanged(QModelIndex,QModelIndex)),
            this,
            SLOT(on_taskList_selection_changed(QModelIndex,QModelIndex)) );


    //ui->task_params_layout->addRow(QString("Tast"), new QLabel("hej med test"));
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
    update();

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
        hideTaskParams();
        return;
    }

    MissionParams params = mh_->getMissionParams();
    ui->mission_name->setText(QString::fromStdString(params.name));
    ui->cad_ref->setText(QString::fromStdString(params.CAD_ref));
}

void MainWindow::update()
{
    updateMissionMeta();
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
}


