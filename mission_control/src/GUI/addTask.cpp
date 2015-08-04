#include <QMessageBox>
#include <QListWidget>
#include "ros/service_client.h"

#include "mission_control/getTaskParams.h"
#include "mission_control/setTaskData.h"
#include "mission_control/ui_api_defines.h"

#include "addTask.hpp"
#include "ui_addTask.h"


using namespace std;

AddTask::AddTask(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddTask)
{
    ui->setupUi(this);


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
    ui->name->setText("");

    ui->distance->setMaximum(srv.response.distance.max_value);
    ui->distance->setMinimum(srv.response.distance.min_value);
    ui->distance->setValue(srv.response.distance.default_value);

    ui->proximity->setMaximum(srv.response.proximity.max_value);
    ui->proximity->setMinimum(srv.response.proximity.min_value);
    ui->proximity->setValue(srv.response.proximity.default_value);

    ui->force->setMaximum(srv.response.force.max_value);
    ui->force->setMinimum(srv.response.force.min_value);
    ui->force->setValue(srv.response.force.default_value);

    ui->power->setMaximum(srv.response.power.max_value);
    ui->power->setMinimum(srv.response.power.min_value);
    ui->power->setValue(srv.response.power.default_value);

    /*
    ui->nav_x_edit->setMinimum(-10000.0);
    ui->nav_x_edit->setMaximum(10000.0);
    ui->nav_x_edit->setValue(ui->nav_x_label->text().toDouble());

    ui->nav_y_edit->setMinimum(-10000.0);
    ui->nav_y_edit->setMaximum(10000.0);
    ui->nav_y_edit->setValue(ui->nav_y_label->text().toDouble());

    ui->nav_yaw_edit->setMinimum(-10000.0);
    ui->nav_yaw_edit->setMaximum(10000.0);
    ui->nav_yaw_edit->setValue(ui->nav_yaw_label->text().toDouble());
    */

    ///Fill the distribution combo-box:
    int default_index = 0; //first one, if default not found. That should, however, not happen.
    for(int i=0;i<(int)srv.response.distributions.values.size();i++)
    {
        ui->distribution->addItem(QString::fromStdString(srv.response.distributions.values[i]));

        //find the default value's index
        if(srv.response.distributions.values[i] == srv.response.distributions.default_value)
            default_index = i;
    }

    //Set index
    ui->distribution->setCurrentIndex(default_index);

    ///Fill the stud type combobox
    default_index = 0;
    for(int i=0;i<(int)srv.response.stud_types.values.size();i++)
    {
        ui->type->addItem(QString::fromStdString(srv.response.stud_types.values[i]));

        //find the default value's index
        if(srv.response.stud_types.values[i] == srv.response.stud_types.default_value)
            default_index = i;

    }

    //Set index
    ui->type->setCurrentIndex(default_index);
}

AddTask::~AddTask()
{
    delete ui;
}

void AddTask::on_buttonBox_accepted()
{
    //send the data/request to the mission controller.
    //go to edit mode:
    mission_control::setTaskData srv;
    srv.request.name = ui->name->text().toStdString();
    srv.request.data.stud_type = ui->type->currentText().toStdString();
    srv.request.data.stud_pattern.distance = ui->distance->value();
    srv.request.data.stud_pattern.proximity = ui->proximity->value();
    srv.request.data.stud_pattern.distribution = ui->distribution->currentText().toStdString();
    srv.request.data.stud_pattern.press = ui->force->value();
    srv.request.data.stud_pattern.power = ui->power->value();

    ros::ServiceClient client = n.serviceClient<mission_control::setTaskData>(UIAPI_ADD_TASK);
    if(!client.call(srv))
        return;

    if(!srv.response.success)
        return;
}



