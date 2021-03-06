#include <QMessageBox>
#include <QListWidget>

#include "ros/node_handle.h"
#include "ros/service_client.h"
#include "loadMission.hpp"
#include "ui_loadMission.h"
#include "../mission_handler.hpp"
#include "mission_control/getMissionList.h"
#include "mission_control/ui_api_defines.h"

using namespace std;

LoadMission::LoadMission(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LoadMission)
{
    ui->setupUi(this);

    //get list of available missions:
    ros::NodeHandle n;
    mission_control::getMissionList srv;
    ros::ServiceClient client = n.serviceClient<mission_control::getMissionList>(UIAPI_GET_MISSION_LIST);
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service");
    }

    vector<string> missions = srv.response.missions;

    //add to list
    for(int i=0;i<(int)missions.size();i++)
    {
            ui->mission_list->addItem(QString::fromStdString( missions[i] ));
    }
}

LoadMission::~LoadMission()
{
    delete ui;
}

void LoadMission::on_buttonBox_accepted()
{
    selected_mission_ = ui->mission_list->currentItem()->text().toStdString();
}



