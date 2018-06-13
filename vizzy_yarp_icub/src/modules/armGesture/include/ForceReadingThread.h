#include <string>
#include <yarp/os/Thread.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <TactSensor.h>
#include <TactSensorArray.h>
#include <yarp/os/Subscriber.h>
#include <yarp/sig/all.h>




class ForceReadingThread : public yarp::os::Thread
{
public:

    ForceReadingThread():semStart(0) { }
    virtual ~ForceReadingThread();
    virtual bool threadInit();
    ForceReadingThread(yarp::os::Subscriber<TactSensorArray> *my_topic__);
    virtual void setSubscriber(yarp::os::Subscriber<TactSensorArray> *my_topic__);
    virtual void run();
    void get_force(int index,yarp::sig::Vector& force);
    virtual void threadRelease();
private:
    yarp::os::Subscriber<TactSensorArray> *my_topic;
    yarp::sig::Vector* image;
    std::vector<TactSensor>* array;
    bool interrupted;
    yarp::os::Semaphore semStart;
    yarp::os::Semaphore semDone;
    yarp::os::Mutex guard;
};

