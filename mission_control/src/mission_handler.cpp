#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED

#include "ros/param.h"
#include "ros/package.h"
#include "ros/node_handle.h"
#include <boost/filesystem.hpp>
#include <sys/stat.h>
#include <sstream>
#include "ros/service_client.h"
#include "std_srvs/Empty.h"
#include "mission_ctrl_msgs/mission_ctrl_defines.h"
#include "mission_control/Trigger.h"
#include "XMLOperations.hpp"

#include "mission_handler.hpp"

#define TEMP_NAMESPACE "temp"

using namespace std;

MissionHandler* MissionHandler::instance_ = NULL;

MissionHandler* MissionHandler::getInstance()
{
    if (!instance_)   // Only allow one instance of class to be generated.
        instance_ = new MissionHandler();

    return instance_;
}

MissionHandler::MissionHandler()
{
    auto_save_ = true;
}

MissionHandler::~MissionHandler()
{
    if(ros::param::has("mission"))
    {
        save();
    }
    close();
}

bool MissionHandler::isLoaded()
{
    return ros::param::has("mission");
}

string MissionHandler::getLoadedName()
{
    if(!isLoaded())
        return "";

    string name;
    ros::param::get("mission/name", name);

    return name;
}

void MissionHandler::close()
{    
    //check if open:
    if(ros::param::has("mission"))
    {
        ros::param::del("mission");
    }
}

void MissionHandler::closeTemp()
{
    //check if open:
    if(ros::param::has(TEMP_NAMESPACE))
    {
        ros::param::del(TEMP_NAMESPACE);
    }
}

bool MissionHandler::load(string name, bool temp)
{
    //close any existing mission:
    if(!temp)
        close();
    else
        closeTemp();

    //check name for extension (we don't expect extension)
    size_t ext_found = name.find(".");
    if(ext_found != string::npos)
    {
        name.erase(ext_found);
    }

    //Check if name exists:
    vector<string> missions = getListOfMissions();

    bool found = false;

    for(int i=0;i<(int)missions.size();i++)
    {
        if(name == missions[i])
        {
            found = true;
            break;
        }
    }

    if(!found)
    {
        ROS_ERROR_STREAM("Requested mission: " << name << " doens't exist in the library!");
        return false;
    }

    //load the mission:
    string system_call_command = "rosparam load " + getStoragePath() + "/" + name + ".yaml /mission";

    if(temp)
        system_call_command = "rosparam load " + getStoragePath() + "/" + name + ".yaml /" + TEMP_NAMESPACE;

    int state = system(system_call_command.c_str());

    return (state == 0 ? true : false);
}

bool MissionHandler::save()
{
    //check that a mission is already loaded:
    if(!ros::param::has("mission"))
    {
        ROS_ERROR("Cannot save mission. No mission loaded.");
        return false;
    }

    //get the mission name:
    string mission_name;
    if(!ros::param::get("mission/name",mission_name))
    {
        ROS_ERROR("Cannot save mission. Loaded mission doesn't have a name");
        return false;
    }

    //update the "last saved tag":
    ros::param::set("mission/last_saved", ros::Time::now().toSec());

    //dump the mission to file:
    string system_call_command = "rosparam dump " + getStoragePath() + "/" + mission_name + ".yaml /mission";
    int state = system(system_call_command.c_str());

    return (state == 0 ? true : false);
}

bool MissionHandler::saveAs(string name)
{   
    //will change the mission name on the param server as well (just like when you save as in any other program)

    //check that a mission is already loaded:
    if(!ros::param::has("mission"))
    {
        ROS_ERROR("Cannot save mission. No mission loaded.");
        return false;
    }

    //change the mission name:
    ros::param::set("mission/name", name);

    return save();
}

bool MissionHandler::createNew(string name)
{
    //close existing:
    close();

    //check if name exists:
    vector<string> missions = getListOfMissions();

    for(int i=0;i<(int)missions.size();i++)
    {
        if(missions[i] == name)
        {
            ROS_ERROR_STREAM("Cannot create new mission of name: " << name << ", beause it already exists.");
            return false;
        }
    }

    //create a new empty mission:
    ros::param::set("mission/name", name);
    ros::param::set("mission/state", mission::EMPTY); //might need to typecast to int... we'll see at compile time

    return save();
}

vector<string> MissionHandler::getListOfMissions()
{
    namespace fs = ::boost::filesystem;

    //get the path of the stored missions:

    vector<string> missions;

    string path = getStoragePath();

    fs::path p(path);

    // return the filenames of all files that have the specified extension
    // in the specified directory and all subdirectories
    //void get_all(const fs::path& root, const string& ext, vector<fs::path>& ret)

    if(!fs::exists(p) || !fs::is_directory(p))
        return missions;

    fs::recursive_directory_iterator it(p);
    fs::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(fs::is_regular_file(*it) && it->path().extension() == ".yaml")
            missions.push_back(it->path().stem().string());                 //stem instead of "filename" to leave out the extension
        ++it;
    }

    return missions;
}

bool MissionHandler::isMission(string name)
{
    //check if "name" is a mission:
    vector<string> missions = getListOfMissions();

    for(int i=0;i<(int)missions.size();i++)
    {
        if(name == missions[i])
            return true;
    }

    return false;
}

bool MissionHandler::isTask(string task_name)
{
    //check index:
    vector<string> tasks = getTaskList();

    bool found = false;

    for(int i=0;i<(int)tasks.size();i++)
    {
        if(tasks[i] == task_name)
        {
            found = true;
            break;
        }
    }

    return found;
}

mission::state MissionHandler::getState()
{
    if(!isLoaded())
    {
        ROS_ERROR("Cannot get mission state. No mission loaded");
        return mission::state(mission::EMPTY);
    }

    int state;
    ros::param::get("mission/state", state);
    return mission::state((mission::states)state);
}

mission::state MissionHandler::getState(string name)
{
    if(!isMission(name))
    {
        ROS_ERROR_STREAM("Failed to get state for mission: " << name << ". Reason: It is not a valid mission");
        return mission::state(mission::EMPTY);
    }

    //load into temp namespace:
    if(!load(name, true))
    {
        ROS_ERROR_STREAM("Failed to load mission: " << name << " temporarily. Cannot get state");
        return mission::state(mission::EMPTY);
    }

    int state;
    ros::param::get( ((string)TEMP_NAMESPACE + "/state"), state);
    return mission::state((mission::states)state);
}

mission::state MissionHandler::getTaskState(string task_name)
{
    if(!isLoaded())
    {
        ROS_ERROR("Cannot get task state. No mission loaded");
        return mission::state(mission::EMPTY);
    }

    //check index:
    vector<string> tasks = getTaskList();

    int index = -1;

    for(int i=0;i<(int)tasks.size();i++)
    {
        if(tasks[i] == task_name)
        {
            index = i;
            break;
        }
    }

    if(index == -1)
    {
        ROS_ERROR_STREAM("Failed to get task state. Taskname: " << task_name << " wasn't found.");
        return mission::state(mission::EMPTY);
    }

    int state;
    ros::param::get(("mission/tasks/" + task_name + "/state"), state);
    return mission::state((mission::states)state);
}

mission::state MissionHandler::getTaskState(int index)
{
    if(!isLoaded())
    {
        ROS_ERROR("Cannot get task state. No mission loaded");
        return mission::state(mission::EMPTY);
    }

    //check index:
    vector<string> tasks = getTaskList();

    if(index > (int)tasks.size()-1 || index < 0)
    {
        ROS_ERROR_STREAM("Failed to get task state. Index: " << index << " is out of range.");
        return mission::state(mission::EMPTY);
    }

    int state;
    ros::param::get(("mission/tasks/" + tasks[index] + "/state"), state);
    return mission::state((mission::states)state);
}

vector<string> MissionHandler::getTaskList()
{
    vector<string> tasks;

    //check if mission is open:
    if(!isLoaded())
    {
        ROS_ERROR("Cannot get task list. No mission loaded.");
        return tasks;
    }

    XmlRpc::XmlRpcValue data;
    if(!ros::param::get("mission/tasks", data))
    {
        ROS_INFO("No tasks in mission");
        return tasks;
    }

    for(XmlRpc::XmlRpcValue::iterator it = data.begin(); it != data.end(); it++)
    {
        tasks.push_back(it->first);
    }

    return tasks;
}

vector<string> MissionHandler::getExecutableTasks()
{
    vector<string> executable_tasks;
    vector<string> all_tasks = getTaskList();

    for(int i=0;i<(int)all_tasks.size();i++)
    {
        if(getTaskState(all_tasks[i]) == mission::INSTRUCTED || getTaskState(all_tasks[i]) == mission::PARTIALLY_COMPLETED)
            executable_tasks.push_back(all_tasks[i]);
    }

    return executable_tasks;
}

vector<string> MissionHandler::getInstructableTasks()
{
    vector<string> instructable_tasks;
    vector<string> all_tasks = getTaskList();

    for(int i=0;i<(int)all_tasks.size();i++)
    {
        if(getTaskState(all_tasks[i]) == mission::CONFIGURED)
            instructable_tasks.push_back(all_tasks[i]);
    }

    return instructable_tasks;
}

TaskData MissionHandler::getTaskData(string task_name)
{
    if(!isLoaded())
    {
        ROS_ERROR("Cannot get task params. No mission loaded");
        return TaskData();
    }

    //check index:
    vector<string> tasks = getTaskList();

    int index = -1;

    for(int i=0;i<(int)tasks.size();i++)
    {
        if(tasks[i] == task_name)
        {
            index = i;
            break;
        }
    }

    if(index == -1)
    {
        ROS_ERROR_STREAM("Failed to get params. Taskname: " << task_name << " wasn't found.");
        return TaskData();
    }

    return getTaskData(index);
}

TaskData MissionHandler::getTaskData(int index)
{
    //Check that a mission is loaded:
    if(!isLoaded())
    {
        ROS_ERROR("Cannot get task params. No mission loaded");
        return TaskData();
    }

    //check index:
    vector<string> tasks = getTaskList();

    if(index > (int)tasks.size()-1 || index < 0)
    {
        ROS_ERROR_STREAM("Failed to get task params. Index: " << index << " is out of range.");
        return TaskData();
    }

    string key = "mission/tasks/" + tasks[index];

    TaskData params;
    params.name = tasks[index];
    ros::param::get((key + "/nav_goal/x"), params.navigation_goal.x);
    ros::param::get((key + "/nav_goal/y"), params.navigation_goal.y);
    ros::param::get((key + "/nav_goal/yaw"), params.navigation_goal.yaw);
    ros::param::get((key + "/stud_type"), params.stud_type);
    ros::param::get((key + "/stud_pattern/distribution"), params.stud_pattern.distribution);
    ros::param::get((key + "/stud_pattern/proximity"), params.stud_pattern.proximity);
    ros::param::get((key + "/stud_pattern/distance"), params.stud_pattern.distance);
    ros::param::get((key + "/stud_pattern/press"), params.stud_pattern.press);
    ros::param::get((key + "/voltage"), params.voltage);

    int temp_direction;
    ros::param::get((key + "/direction"), temp_direction);
    params.direction = (unsigned int)temp_direction;

    int state;
    ros::param::get((key + "/state"), state);
    params.state = (mission::states)state;

    //Check if task has studs:
    if(!ros::param::has( (key + "/studs") ) )
        return params;

    XmlRpc::XmlRpcValue data;
    ros::param::get( (key + "/studs"),data );

    for(XmlRpc::XmlRpcValue::iterator it = data.begin(); it != data.end(); it++)
    {
        StudPosition stud_pos;

        ros::param::get( (key + "/studs/" + it->first + "/x"), stud_pos.x );
        ros::param::get( (key + "/studs/" + it->first + "/y"), stud_pos.y );
        ros::param::get( (key + "/studs/" + it->first + "/state"), stud_pos.state );
        ros::param::get( (key + "/studs/" + it->first + "/time_stamp"), stud_pos.time_stamp );

        params.studs.push_back(stud_pos);
    }

    return params;
}

MissionData MissionHandler::getMissionData()
{
    MissionData params;

    if(!isLoaded())
    {
        ROS_ERROR("Failed to get mission parameters. No mission loaded");
        return params;
    }

    params.name = getLoadedName();
    ros::param::get("mission/CAD", params.CAD_ref);
    ros::param::get("mission/description", params.description);
    ros::param::get("mission/last_saved", params.last_saved);

    int state;
    ros::param::get("mission/state", state);
    params.state = (mission::states)state;

    vector<string> tasks = getTaskList();
    params.number_of_tasks = (int)tasks.size();

    return params;
}

MissionData MissionHandler::getMissionData(string name)
{
    MissionData params;

    if(!isMission(name))
    {
        ROS_ERROR_STREAM("Failed to get mission parameters for mission: " << name << ". Reason: It is not a valid mission");
        return params;
    }

    //load into temp namespace:
    if(!load(name, true))
    {
        ROS_ERROR_STREAM("Failed to load mission: " << name << " temporarily. Cannot get params");
        return params;
    }

    ros::param::get( ((string)TEMP_NAMESPACE + "/CAD"), params.CAD_ref);
    ros::param::get( ((string)TEMP_NAMESPACE + "/name"), params.name);
    ros::param::get( ((string)TEMP_NAMESPACE + "/last_saved"), params.last_saved);
    ros::param::get( ((string)TEMP_NAMESPACE + "/description"), params.description);

    int state;
    ros::param::get( ((string)TEMP_NAMESPACE + "/state"), state);
    params.state = (mission::states)state;

    //get number of tasks:
    XmlRpc::XmlRpcValue data;
    if(!ros::param::get(((string)TEMP_NAMESPACE + "/tasks"), data))
        params.number_of_tasks = 0;
    else
        params.number_of_tasks = data.size();

    closeTemp();

    return params;
}

vector<string> MissionHandler::getStudList(string task_name)
{
    vector<string> studs;

    if(!isLoaded())
    {
        ROS_ERROR("Cannot get stud list. No mission loaded");
        return studs;
    }

    //check that task exists:
    vector<string> tasks = getTaskList();

    if(!ros::param::has(("mission/tasks/" + task_name)))
    {
        ROS_ERROR_STREAM("Failed to get stud list. Task: " << task_name << " doesn't exist in mission: " << getLoadedName());
        return studs;
    }

    //get the stud list:
    XmlRpc::XmlRpcValue data;
    if(!ros::param::get(("mission/tasks/" + task_name + "/studs"), data))
    {
        ROS_DEBUG_STREAM("No studs in task: " << task_name);
        return studs;
    }

    for(XmlRpc::XmlRpcValue::iterator it = data.begin(); it != data.end(); it++)
    {
        studs.push_back(it->first);
    }

    return studs;
}

vector<string> MissionHandler::getPendingStuds(string task_name)
{
    vector<string> all_studs = getStudList(task_name);
    vector<string> pending_studs;

    for(int i=0;i<(int)all_studs.size();i++)
    {
        //check if stud is pending:
        int stud_state;
        ros::param::get(("mission/tasks/" + task_name + "/studs/" + all_studs[i] + "/state"),stud_state);
        if(stud_state == (int)stud::PENDING)
            pending_studs.push_back(all_studs[i]);
    }

    return pending_studs;
}

int MissionHandler::getDirection(string task_name)
{
    //check that a mission is loaded:
    if(!isLoaded())
    {
        ROS_ERROR_STREAM("Cannot get direction parameter. No mission loaded.");
        return -1;
    }

    //check that task exists:
    vector<string> tasks = getTaskList();

    if(!ros::param::has(("mission/tasks/" + task_name)))
    {
        ROS_ERROR_STREAM("Failed to get direction. Task: " << task_name << " doesn't exist in mission: " << getLoadedName());
        return -1;
    }

    int direction = -1;

    ros::param::get(("mission/tasks/" + task_name + "/studs"), direction);

    return direction;
}

bool MissionHandler::setMissionData(mission_control::MissionData data)
{
    //check that a mission is loaded:
    if(!isLoaded())
    {
        ROS_ERROR_STREAM("Cannot set mission data. No mission loaded.");
        return false;
    }

    string current_name = getLoadedName();

    //check if the name has changed:
    if(data.name != "" && data.name != current_name)
    {
        namespace fs = boost::filesystem;

        //check that the new name is not conflicting with an already existing mission:
        vector<string> missions = getListOfMissions();
        for(int i=0;i<(int)missions.size();i++)
        {
            if(data.name == missions[i])
            {
                ROS_ERROR_STREAM("Cannot change mission data. New name: " << data.name << " conflicts with already existing mission name");
                return false;
            }
        }

        //the name has been changed (and a new name is provided)'
        ros::param::set("mission/name", data.name);

        //call save to save the file under the new name:
        save();

        //now delete the "old" file:
        string old_file = getStoragePath() + "/" + current_name + ".yaml";
        struct stat st;
        lstat(old_file.c_str(), &st);
        if(S_ISDIR(st.st_mode))
        {
            std::cerr << "Error: Tried to delete directory! Aborting. " << std::endl;
            return false;
        }

        fs::remove(old_file.c_str());
    }

    //update/set the data:
    ros::param::set("mission/CAD", data.cad);
    ros::param::set("mission/description", data.description);

    //save
    save();

    return true;
}

bool MissionHandler::addStud(string task_name, double x, double y, string stud_name)
{
    if(getTaskState(task_name).toUInt() > mission::INSTRUCTED || getTaskState(task_name).toUInt() < mission::CONFIGURED)
    {
        ROS_ERROR_STREAM("Failed to add stud to task " << task_name << ". Task either not configured or already (partly) completed");
        return false;
    }

    if(stud_name == "")
    {
        vector<string> studs = getStudList(task_name);
        int index = 0;
        for(int i=0;i<(int)studs.size();i++)
        {
            string current_index = studs[i].substr(studs[i].find("_")+1);
            int temp_index = atoi(current_index.c_str());
            if(temp_index > index)
                index = temp_index;
        }

        stringstream ss;
        ss << "stud_" << index+1;
        stud_name = ss.str();
    }

    string key = "mission/tasks/" + task_name + "/studs/" + stud_name + "/";
    ros::param::set((key + "state"), (int)stud::PENDING);
    ros::param::set((key + "x"), x);
    ros::param::set((key + "y"), y);
    ros::param::set((key + "time_stamp"), (double)ros::Time::now().toSec());

    return true;
}

bool MissionHandler::setStudState(string task_name, string stud_name, stud::states state)
{
    ROS_DEBUG("MissionHandler::setStudState");

    //update the stud state:
    if(!ros::param::has("/mission/tasks/" + task_name + "/studs/" + stud_name))
    {
        ROS_ERROR_STREAM("Failed to set state for stud: " << stud_name << " in task: " << task_name << ". Stud doesn't exist.");
        return false;
    }

    ros::param::set(("/mission/tasks/" + task_name + "/studs/" + stud_name + "/state"), (int)state);
    ros::param::set(("/mission/tasks/" + task_name + "/studs/" + stud_name + "/time_stamp"), ros::Time::now().toSec());

    updateTaskState(task_name);

    return true;
}

bool MissionHandler::setTaskParamsDefault(string task_name)
{
    ROS_DEBUG_STREAM("MissionHandler::setTaskParamsDefault - task_name: " << task_name);

    //open the task_param_database.xml to find the available parameters. It is located in the root of the mission_control package)
    XMLOperations xmlop(false);
    if(!xmlop.openFile(ros::package::getPath("mission_control"), "task_params_database.xml"))
    {
        ROS_ERROR_STREAM("Failed to open task parameter database");
        return false;
    }

    //key to the task namespace on paramserver
    string key = "mission/tasks/" + task_name;

    ///UPDATE TASK PARAMS TO DEFAULT

    //stud type:
    xmlop.goToFirstMatch("stud_type");
    ros::param::set((key + "/stud_type"), xmlop.getAttribute("default").toString());

    //distribution
    xmlop.goToFirstMatch("stud_distribution");
    ros::param::set((key + "/stud_pattern/distribution"), xmlop.getAttribute("default").toString());

    //distance
    xmlop.goToFirstMatch("stud_distance");
    ros::param::set((key + "/stud_pattern/distance"), xmlop.getAttribute("default").toDouble());


    //proximity
    xmlop.goToFirstMatch("stud_proximity");
    ros::param::set((key + "/stud_pattern/proximity"), xmlop.getAttribute("default").toDouble());

    //force
    xmlop.goToFirstMatch("force");
    ros::param::set((key + "/stud_pattern/press"), xmlop.getAttribute("default").toDouble());

    //power
    xmlop.goToFirstMatch("voltage");
    ros::param::set((key + "/voltage"), xmlop.getAttribute("default").toDouble());

    //direction
    xmlop.goToFirstMatch("direction");
    ros::param::set((key + "/direction"), xmlop.getAttribute("default").toInt());

    //update the task state:
    return updateTaskState(task_name);
}

bool MissionHandler::autoGenTasks()
{
    ROS_DEBUG("MissionHandler::autoGenTasks");

    //check if mission is loaded
    if(!isLoaded())
        return false;

    if(getTaskList().size() > 0)   //hence, contains tasks
    {
        ROS_WARN("Cannot auto-generate tasks. Mission already contains tasks.");
        return false;
    }

    ros::NodeHandle n;
    std_srvs::Empty srv;
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>(CARLOS_INIT_GOAL_GEN_SRV);

    if(!client.waitForExistence(ros::Duration(2.0)))
    {
        ROS_ERROR("Auto-generate tasks service not available. Timeout on connection.");
        return false;
    }

    if(!client.call(srv))
    {
        ROS_ERROR("Auto-generation of tasks failed");
        return false;
    }

    //tasks has been generated. Now do set all params to "default" for all tasks
    vector<string> tasks = getTaskList();
    for(int i=0;i<(int)tasks.size();i++)
    {
        setTaskParamsDefault(tasks[i]);
    }

    return true;
}

bool MissionHandler::updateMissionState()
{
    ROS_DEBUG("MissionHandler::updateMissionState");

    //update the mission state based on data:

    if(!isLoaded())
    {
        ROS_ERROR("Cannot update mission state. No mission loaded");
        return false;
    }

    vector<string> task_list = getTaskList();
    mission::states state = mission::COMPLETED;
    vector<mission::states> states;


    //setting the "state" equal to the lowest task state:
    for(int i=0;i<(int)task_list.size();i++)
    {
        states.push_back((mission::states)getTaskState(i).toUInt());
    }

    if(states.size() == 0)
    {
        ros::param::set("mission/state", (int)mission::EMPTY);
        return true;
    }

    //sort the list of states, so that the lowest state is first:
    sort(states.begin(),states.end());

    //set the state to the lowest state:
    state = states.front();

    /* Now, if the depending on the current state, if
     * there is a higher state, the task could potentially
     * be of 1 higher state. E.G state = instructed, but there
     * is at least one task of state completed or partially completed
     * In general, if the state if NOT partially_XXX, there is an option
     * to be one state  higher. Sounds complicated, but I got it (at least)
     */

    if(state == mission::CONFIGURED || state == mission::INSTRUCTED)
    {
        for(int i=0;i<(int)states.size();i++)
        {
            if(states[i] > state)
            {
                state = (mission::states)((int)state + 1);
                break;
            }
        }
    }

    ros::param::set("mission/state", (int)state);

    if(auto_save_)
        save();

    return true;
}

bool MissionHandler::setTaskData(string name, mission_control::TaskData data)
{
    //get name of loaded mission
    string mission_name = getLoadedName();

    //check that a mission is loaded:
    if(mission_name == "")
    {
        ROS_ERROR_STREAM("Cannot set task data. No mission loaded");
        return false;
    }

    //check if task exists
    if(!isTask(name))
    {
        ROS_ERROR_STREAM("Cannot set task data for task " << name << ". Task not found in current mission " << mission_name);
        return false;
    }

    //check if task execution has already been tried:
    mission::state task_state = getTaskState(name);
    if(task_state.toUInt() > mission::INSTRUCTED)   //task already partially welded
    {
        ROS_ERROR_STREAM("Task " << name << " already partially or fully welded. Changing task data not allowed!. Task state: " << task_state.toString());
        return false;
    }

    //if any of the task params related to "studs" are changed, then any existing studs will be removed!
    TaskData current_data = getTaskData(name);
    if(     data.stud_type!=current_data.stud_type ||
            data.stud_pattern.distance != current_data.stud_pattern.distance ||
            data.stud_pattern.proximity != current_data.stud_pattern.proximity ||
            data.stud_pattern.distribution != current_data.stud_pattern.distribution)       //we leave out "press" as it is only effective during welding.
    {
        //somthing related to the stud positions has changed. Remove all studs!
        ros::param::del(("mission/tasks/" + name + "/studs"));
    }

    //set the task data:
    string key = "mission/tasks/" + name;
    ros::param::set((key + "/nav_goal/x"), data.nav_goal.x);
    ros::param::set((key + "/nav_goal/y"), data.nav_goal.y);
    ros::param::set((key + "/nav_goal/yaw"), data.nav_goal.yaw);
    ros::param::set((key + "/stud_type"), data.stud_type);
    ros::param::set((key + "/stud_pattern/distribution"), data.stud_pattern.distribution);
    ros::param::set((key + "/stud_pattern/proximity"), data.stud_pattern.proximity);
    ros::param::set((key + "/stud_pattern/distance"), data.stud_pattern.distance);
    ros::param::set((key + "/stud_pattern/press"), data.stud_pattern.press);
    ros::param::set((key + "/voltage"), data.voltage);
    ros::param::set((key + "/direction"), data.direction);

    //update the task state:
    return updateTaskState(name);
}

bool MissionHandler::updateTaskState(string task_name)
{
    ROS_DEBUG("MissionHandler::updateTaskState");

    mission::states task_state;

    if(!isLoaded())
    {
        ROS_ERROR("Cannot update task state. No mission loaded");
        return false;
    }

    if(!ros::param::has(("mission/tasks/" + task_name)))
    {
        ROS_ERROR_STREAM("Failed to update task state. Task: " << task_name << " doesn't exist in mission: " << getLoadedName());
        return false;
    }

    //the task state is both given by the task params AND the actual studs. Without the studs, the task state can maximum be "configured"æ
    task_state = mission::CONFIGURED;

    string key = "mission/tasks/" + task_name;
    int found = 0;
    if(!ros::param::has((key + "/nav_goal/x"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}
    if(!ros::param::has((key + "/nav_goal/y"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}
    if(!ros::param::has((key + "/nav_goal/yaw"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}
    if(!ros::param::has((key + "/stud_type"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}
    if(!ros::param::has((key + "/stud_pattern/distribution"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}
    if(!ros::param::has((key + "/stud_pattern/proximity"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}
    if(!ros::param::has((key + "/stud_pattern/distance"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}
    if(!ros::param::has((key + "/direction"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}
    if(!ros::param::has((key + "/voltage"))) {task_state = mission::PARTIALLY_CONFIGURED;} else {found++;}


    if(found == 0)
        task_state = mission::EMPTY;

    if(found < 9)                  //hence, we not all parameters are fully configured - no need to check studs!
    {
        ros::param::set(("mission/tasks/" + task_name + "/state"), (int)task_state);
        return true;
    }

    ///ALL PARAMS ARE CONFIGURED - CHECK FOR STUDS

    //get all the stud states:
    vector<string> studs = getStudList(task_name);

    vector<stud::states> states;
    for(int i=0;i<(int)studs.size();i++)
    {
        int temp;
        ros::param::get(("mission/tasks/" + task_name + "/studs/" + studs[i] + "/state"), temp);
        states.push_back((stud::states)temp);
    }

    if(states.size() > 0) //if at least one stud exists!
    {
        task_state = mission::PARTIALLY_INSTRUCTED;

        sort(states.begin(),states.end());

        if(states.back() == stud::PENDING)
            task_state = mission::INSTRUCTED;
        else if(states.back() == stud::FAILED || states.back() == stud::SUCCEEDED)
            task_state = mission::PARTIALLY_COMPLETED;

        if(states.front() == stud::FAILED || states.front() == stud::SUCCEEDED)
            task_state = mission::COMPLETED;

    }

    ros::param::set(("mission/tasks/" + task_name + "/state"), (int)task_state);

    //update mission state, as changes to task state could affect mission state: (will also save)
    return updateMissionState();
}

bool MissionHandler::addTask(mission_control::TaskData data, string name)
{
    //check that IF a name is given, it does not conflict with existing task name:
    if(isTask(name))
    {
        ROS_ERROR_STREAM("Cannot add task " << name << ". A task with this name already exists");
        return false;
    }

    //if no name given, auto-generate one:
    if(name == "")
    {
        vector<string> tasks = getTaskList();

        int index = 1;
        string task_base = "task_";

        while(true)
        {
            stringstream task_name_ss;
            task_name_ss << task_base << index;
            bool found = false;

            for(int i=0;i<(int)tasks.size();i++)
            {
                if(tasks[i] == task_name_ss.str())
                {
                    found = true;
                    break;
                }
            }

            if(found)
            {
                index++;
            }
            else            //not found - we have a valid name
            {
                name = task_name_ss.str();
                break;
            }
        } //end while
    } //end if

    //add the task - we do this by adding a state!
    ros::param::set("mission/tasks/" + name + "/state", (int)mission::EMPTY);

    //finally - add the task data.
    return setTaskData(name,data);
}

bool MissionHandler::deleteTask(string task_name)
{
    //check if a task_name is supplied:
    if(task_name == "")
    {
        ROS_ERROR("Cannot delete task. No task name provided");
        return false;
    }

    //check if task_name is valid:
    if(!MissionHandler::getInstance()->isTask(task_name))
    {
        ROS_ERROR_STREAM("Cannot delete task " << task_name << ". Not a valid task");
        return false;
    }

    //delete the task:
    if(!ros::param::del(("mission/tasks/" + task_name)))
        return false;

    //update mission state: (deleting a task could potentially change the mission state!
    updateMissionState();

    //save? - updating mission state just did it for us!

    return true;
}

string MissionHandler::getStoragePath()
{
    return (ros::package::getPath("mission_control") +"/mission_library");
}

string NavGoal::toString(bool verbose)
{
    stringstream ss;

    if(!verbose)
        ss << x << " " << y << " " << yaw;
    else
        ss << "x: " << x << " y: " << y << " yaw: " << yaw;

    return ss.str();
}

mission_control::MissionData MissionData::toMsg() const
{
    mission_control::MissionData data;
    data.name = this->name;
    data.description = this->description;
    data.cad = this->CAD_ref;
    data.last_saved = this->last_saved;
    data.number_of_tasks = this->number_of_tasks;
    data.state = this->state.toUInt();
    data.state_description = this->state.toString();

    return data;
}

mission_control::TaskData TaskData::toMsg() const
{
    mission_control::TaskData data;

    data.nav_goal.x = this->navigation_goal.x;
    data.nav_goal.y = this->navigation_goal.y;
    data.nav_goal.yaw = this->navigation_goal.yaw;
    data.number_of_studs = (int)this->studs.size();
    data.state = this->state.toUInt();
    data.state_description = this->state.toString();
    data.stud_type = this->stud_type;
    data.stud_pattern = this->stud_pattern;
    data.direction = this->direction;
    data.voltage = this->voltage;

    return data;
}

