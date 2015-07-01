
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

struct StudPattern  //could be replaced by StudPattern.msg - yet here we could add functions.
{
    std::string distribution;
    double distance;
    double proximity;
    double force;
};

struct TaskParams
{
    std::string name;
    mission::state state;   //for now we adopt the same definition as for a mission
    std::string stud_type;
    StudPattern stud_pattern;
    NavGoal navigation_goal;
    std::vector<StudPosition> studs;
};

struct MissionParams
{
    std::string name;
    std::string CAD_ref;
    std::string description;
    int number_of_tasks;
    double last_saved;
    mission::state state;

};

class MissionHandler
{
    friend class SystemEngine;

public:
    static MissionHandler* getInstance();

    ~MissionHandler();

    void setAutoSave(bool auto_save) {auto_save_ = auto_save;}
    bool getAutoSave() const {return auto_save_;}

    bool isLoaded();

    std::string getLoadedName();

    bool load(std::string name, bool temp = false);

    bool save(void);

    bool saveAs(std::string name);

    bool createNew(std::string name);

    std::vector<std::string> getListOfMissions(void);

    bool isMission(std::string name);

    //get state of mission
    mission::state getState();
    mission::state getState(std::string name);

    mission::state getTaskState(std::string task_name);
    mission::state getTaskState(int index);

    std::vector<std::string> getTaskList();

    std::vector<std::string> getExecutableTasks();
    std::vector<std::string> getInstructableTasks();

    TaskParams getTaskParams(std::string task_name);
    TaskParams getTaskParams(int index);

    MissionParams getMissionParams();
    MissionParams getMissionParams(std::string name);

    std::vector<std::string> getStudList(std::string task_name);

    bool addStud(std::string task_name, double x, double y, std::string stud_name = "auto_generate");
    bool setStudState(std::string task_name, std::string stud_name, stud::states state);

    bool updateMissionState();

    bool updateTaskState(std::string task_name);

private:

    MissionHandler();
    static MissionHandler* instance_;

    void close();
    void closeTemp();

    std::string getStoragePath();
    void lock()     {lock_ = true;}
    void unlock()   {lock_ = false;}

    bool auto_save_;
    bool lock_;

};

#endif /* MISSIONHANDLER_HPP_ */
