#ifndef SYSTEMENGINE_HPP_
#define SYSTEMENGINE_HPP_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/assign.hpp>
#include <map>

class SysState
{

public:
    enum values
    {
        IDLE,
        EXECUTING,
        INSTRUCTING,
        ASSISTING
    };

    SysState()
    {
        compare_map_ = boost::assign::map_list_of
                (IDLE,                   "idle")
                (EXECUTING,               "executing")
                (INSTRUCTING,            "instructing")
                (ASSISTING,              "assisting");
    }

    void operator=( const std::string value_string )
    {
        for(std::map<values, std::string>::const_iterator it = compare_map_.begin();it!=compare_map_.end();it++)
        {
            if(value_string == it->second)
            {
                value_ = it->first;
                return;
            }
        }
        std::cout << "Trying to assign unknown value: " << value_string << ". Value cannot be set!" << std::endl;
    }

    void operator=( const values value )
    {
        value_ = value;
    }

    operator int() {return (int)value_;}
    operator unsigned int() {return (unsigned int)value_;}
    operator std::string() {return toString();}
    operator values() {return value_;}

    std::string toString() const
    {
        if(compare_map_.find(value_) == compare_map_.end())
            return "error";
        return compare_map_.find(value_)->second;
    }


    inline bool operator==(const SysState& other) const { return (other.value_ == this->value_); }
    inline bool operator!=(const SysState& other) const { return !(*this == other); }
    inline bool operator==(const std::string& other) const { return (other == this->toString()); }
    inline bool operator!=(const std::string& other) const { return !(*this == other); }
    inline bool operator==(const values& other) const { return (other == this->value_); }
    inline bool operator!=(const values& other) const { return !(*this == other); }


private:
    values value_;

    std::map<values, std::string> compare_map_;
};

class SystemEngine
{
public:
    static SystemEngine* getInstance();

    bool execute();
    bool instruct();
    bool assist();
    bool edit();

    void executeDone();
    void instructDone();
    void assistDone();
    bool editDone();

    void lockMissionHandler();
    void unlockMissionHandler();

    //current state:
    SysState current_state_;

private:

    struct SysStateMachine;
    boost::shared_ptr<SysStateMachine> ssm_;

    SystemEngine();
    ~SystemEngine();

    void init();

    static SystemEngine* instance_;

};

#endif // SYSTEMENGINE_HPP_
