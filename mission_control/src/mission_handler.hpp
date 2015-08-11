/* Created by Casper Schou @ AAU 2015
 *
 * This class handles data related to the mission and tasks.
 * It handles all writing and reading from the parameter server,
 * and saving and loading to/from file.
 * It implements a singleton pattern to ensure no duplicates.
 */

#ifndef MISSIONHANDLER_HPP_
#define MISSIONHANDLER_HPP_

//=================================
// Forward declared dependencies
//=================================

//=================================
// Included dependencies
//=================================
#include "ros/console.h"
#include <string>
#include <vector>
#include "boost/assign.hpp"
#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include "mission_control/MissionData.h"
#include "mission_control/TaskData.h"

/* Making a wrapper-class of the enum for mission state.
 * This allows for inline conversion to/from unit8 and string + comparisons.
 */
namespace mission
{
class state
{
public:

    state()
    {
        compare_map_ = boost::assign::map_list_of
                (mission::EMPTY,                    "empty")
                (mission::PARTIALLY_CONFIGURED,     "partially_configured")
                (mission::CONFIGURED,               "configured")
                (mission::PARTIALLY_INSTRUCTED,     "partially_instructed")
                (mission::INSTRUCTED,               "instructed")
                (mission::PARTIALLY_COMPLETED,      "partially_completed")
                (mission::COMPLETED,                "completed");
    }

    state(mission::states s) : state_(s) {}

    void operator=( const std::string state_string )
    {
        for(std::map<mission::states, std::string>::const_iterator it = compare_map_.begin();it!=compare_map_.end();it++)
        {
            if(state_string == it->second)
            {
                state_ = it->first;
                return;
            }
        }
        ROS_WARN_STREAM("Trying to assign unknown state: " << state_string);
    }

    void operator=( const mission::states state )
    {
        state_ = state;
    }

    std::string toString() const
    {
        if(compare_map_.find(state_) == compare_map_.end())
            return "unknown";
        return compare_map_.find(state_)->second;
    }

    unsigned int toUInt() const
    {
        return (unsigned int)state_;
    }

    inline bool operator==(const state& other) const { return (other.state_ == this->state_); }
    inline bool operator!=(const state& other) const { return !(*this == other); }
    inline bool operator==(const std::string& other) const { return (other == this->toString()); }
    inline bool operator!=(const std::string& other) const { return !(*this == other); }
    inline bool operator==(const mission::states& other) const { return (other == this->state_); }
    inline bool operator!=(const mission::states& other) const { return !(*this == other); }

private:
    mission::states state_;

    std::map<mission::states, std::string> compare_map_;

};

} //end namespace

/* Below is various data-container-structs used internally.
 * These structs do duplicate some of the ros msg types,
 * however, I decided to keep these as they are extensively used
 * internally in the class. Though, it should be straight forward to
 * exchange the structs with the ros msg types.
 */
struct StudPosition
{
    double x;
    double y;
    std::string state;
    double time_stamp;       //could be another type!
};

struct NavGoal
{
    std::string toString(bool verbose = false);

    double x;
    double y;
    double yaw;
};

struct TaskData
{
    mission_control::TaskData toMsg() const;

    std::string name;
    mission::state state;   //for now we adopt the same definition as for a mission
    std::string stud_type;
    mission_control::StudPattern stud_pattern;
    NavGoal navigation_goal;
    std::vector<StudPosition> studs;
    unsigned int direction;
    double voltage;
};

struct MissionData
{
    mission_control::MissionData toMsg() const;

    std::string name;
    std::string CAD_ref;
    std::string description;
    int number_of_tasks;
    double last_saved;
    mission::state state;
};

class MissionHandler
{

public:
    static MissionHandler* getInstance();

    ~MissionHandler();

    //Functions to set and get the auto_save flag. If auto save is enabled (default)
    //the mission is automatically saved to file on each change.
    void setAutoSave(bool auto_save) {auto_save_ = auto_save;}
    bool getAutoSave() const {return auto_save_;}

    //true if a mission is currently loaded on the param server
    bool isLoaded();

    //get name of loaded mission. If no mission loaded, return empty string
    std::string getLoadedName();

    /* Load a mission from the mission library with the given name.
     * If temp is set true, the mission is loaded into a temp namespace, hence
     * it doesn't replace any previously loaded mission.
     * The point of the temp namespace is to be able to read the data of a
     * mission without having to replace any already loaded mission.
     */
    bool load(std::string name, bool temp = false);

    //Save a mission to file. Missions are stored as .yaml in the mission_library folder.
    bool save(void);

    //Save mission as another name.
    bool saveAs(std::string name);

    //create a new mission with the given name
    bool createNew(std::string name);

    //get a list of available missions
    std::vector<std::string> getListOfMissions(void);

    //check if the given name corresponds to a mission in the library
    bool isMission(std::string name);

    //check if the given task_name matches any of the tasks in the currently loaded mission
    bool isTask(std::string task_name);

    //get state of mission
    mission::state getState();
    mission::state getState(std::string name);

    //get state of task
    mission::state getTaskState(std::string task_name);
    mission::state getTaskState(int index);

    //get list of tasks
    std::vector<std::string> getTaskList();
    std::vector<std::string> getExecutableTasks();
    std::vector<std::string> getInstructableTasks();

    //get data for task
    TaskData getTaskData(std::string task_name);
    TaskData getTaskData(int index);

    //get mission data
    MissionData getMissionData();
    MissionData getMissionData(std::string name);

    //get stud lists
    std::vector<std::string> getStudList(std::string task_name);
    std::vector<std::string> getPendingStuds(std::string task_name);

    //modify mission data
    bool setMissionData(mission_control::MissionData data);
    bool updateMissionState();

    //modify task data
    bool setTaskData(std::string name, mission_control::TaskData data);

    /* Trigger the system to update the task state.
     * Determining the task state is done automatically by the system.
     */
    bool updateTaskState(std::string task_name);

    /* add a task to the currently loaded mission.
     * If no name is given, a name is auto-generated
     * following the "task_<number>" convension, choosing the
     * first available number. (counting from 1)
     * BTW, not sure AIMEN can actually handle tasks with names
     * NOT following the "task_<number>" convension.
     */
    bool addTask(mission_control::TaskData data, std::string name = "");

    //delete task
    bool deleteTask(std::string task_name);

    /* add a stud to the given task in the currently loaded mission.
     * If no name is given, a name is auto-generated
     * following the "stud_<number>" convension, choosing the
     * first available number. (counting from 1)
     */
    bool addStud(std::string task_name, double x, double y, std::string stud_name = "");

    //change the state of a stud.
    bool setStudState(std::string task_name, std::string stud_name, stud::states state);

private:

    MissionHandler();
    static MissionHandler* instance_;

    //"close" mission - hence remove from param server
    void close();
    void closeTemp();

    //get path to the mission library - it is a folder in the mission_control package
    std::string getStoragePath();

    //auto save flag - usage described above.
    bool auto_save_;

};

#endif /* MISSIONHANDLER_HPP_ */
