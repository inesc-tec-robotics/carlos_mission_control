#ifndef SYSTEMENGINE_HPP_
#define SYSTEMENGINE_HPP_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

class SystemEngine
{
public:
    static SystemEngine* getInstance();

    bool execute();
    void executeDone();
    void instructDone();
    void assistDone();

    void lockMissionHandler();
    void unlockMissionHandler();

private:

    struct SysStateMachine;
    boost::shared_ptr<SysStateMachine> ssm_;

    SystemEngine();

    ~SystemEngine();

    static SystemEngine* instance_;

};

#endif // SYSTEMENGINE_HPP_
