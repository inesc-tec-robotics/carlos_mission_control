/* Created by Casper Schou @ AAU 2015
 *
 */

#ifndef LOADMISSION_HPP_
#define LOADMISSION_HPP_

#include <QDialog>

namespace Ui {
class LoadMission;
}

class LoadMission : public QDialog
{
	Q_OBJECT
	
public:
    explicit LoadMission(QWidget *parent = 0);
    ~LoadMission();

    std::string getSelectedMission() const { return selected_mission_; }
	
private Q_SLOTS:
	void on_buttonBox_accepted();

private:
    Ui::LoadMission *ui;

    std::string selected_mission_;
};

#endif // LOADMISSION_HPP_
