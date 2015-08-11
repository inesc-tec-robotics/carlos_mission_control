/* Created by Casper Schou @ AAU 2015
 *
 */

#ifndef ADDTASK_HPP_
#define ADDTASK_HPP_

#include <QDialog>
#include "ros/node_handle.h"


namespace Ui {
class AddTask;
}

class AddTask : public QDialog
{
	Q_OBJECT
	
public:
    explicit AddTask(QWidget *parent = 0);
    ~AddTask();

private Q_SLOTS:
	void on_buttonBox_accepted();

private:
    Ui::AddTask *ui;
    ros::NodeHandle n;

};

#endif // ADDTASK_HPP_
