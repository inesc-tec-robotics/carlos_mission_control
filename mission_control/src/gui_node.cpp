#include <QtCore>
#include "GUI/mainwindow.h"
#include "GUI/ros_interface.hpp"

using namespace std;

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "mission_controller_gui");

    //create main window object
    QApplication a(argc, argv);
    setlocale(LC_NUMERIC,"C");
    MainWindow w;

    //Create RosInterface object
    RosInterface ros_interface;

    //Connect the ros interface signals with the GUI slots. (Hence, this is how we send events from ros interface to the gui)
    qRegisterMetaType<mission_control::Progress::ConstPtr>("mission_control::Progress::ConstPtr");
    qRegisterMetaType<mission_control::HardwareStates>("mission_control::HardwareStates");
    QObject::connect(&ros_interface, SIGNAL(exec_progress_update_received(const mission_control::Progress::ConstPtr&)), &w, SLOT(execProgressUpdate(const mission_control::Progress::ConstPtr&)));
    QObject::connect(&ros_interface, SIGNAL(instr_progress_update_received(const mission_control::Progress::ConstPtr&)), &w, SLOT(instrProgressUpdate(const mission_control::Progress::ConstPtr&)));
    QObject::connect(&ros_interface, SIGNAL(hw_states_received(mission_control::HardwareStates)), &w, SLOT(hwStateUpdate(mission_control::HardwareStates)));

    //start the ros interface in a separate thread:
    ros_interface.start();

    //show the main window
    w.show();

    //execute the application.
    a.exec();

    return 0;
}
