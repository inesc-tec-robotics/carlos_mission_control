/* Created by Casper Schou @ AAU 2015
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

    void on_gen_tasks_button_clicked();

    void on_execKillButton_clicked();

    void on_instrKillButton_clicked();

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

    //enter/exit editing state
    bool enterEditMode();
    bool exitEditMode();

    Ui::MainWindow *ui;

    //QModel to hold the data for the task list.
    QStandardItemModel* task_list_model;

    ros::NodeHandle n;

    //subscribers to the hardware states and the progress updates
    ros::Subscriber hw_state_sub;
    ros::Subscriber exec_progress_sub;
    ros::Subscriber instr_progress_sub;

};

#endif // MAINWINDOW_H
