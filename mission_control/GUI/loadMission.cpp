#include <QMessageBox>
#include <QListWidget>

#include "loadMission.hpp"
#include "ui_loadMission.h"
#include "../mission_handler.hpp"

using namespace std;

LoadMission::LoadMission(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LoadMission)
{
    ui->setupUi(this);

    //get list of available missions:
    vector<string> missions = MissionHandler::getInstance()->getListOfMissions();

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



