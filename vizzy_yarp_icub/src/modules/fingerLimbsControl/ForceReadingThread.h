#include <string>
#include <yarp/os/Thread.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <vizzy_tactile_TactSensor.h>
#include <vizzy_tactile_TactSensorArray.h>
#include <yarp/os/Subscriber.h>
#include <yarp/sig/all.h>




class ForceReadingThread : public yarp::os::Thread
{
public:

    ForceReadingThread():semStart(0) { }
    virtual ~ForceReadingThread();
    virtual bool threadInit();
    ForceReadingThread(yarp::os::Subscriber<vizzy_tactile_TactSensorArray> *my_topic__);
    virtual void setSubscriber(yarp::os::Subscriber<vizzy_tactile_TactSensorArray> *my_topic__);
    virtual void run();
    void get_force(yarp::sig::Vector& force);
    virtual void threadRelease();
private:
    yarp::os::Subscriber<vizzy_tactile_TactSensorArray> *my_topic;
    yarp::sig::Vector* image;
    geometry_msgs_Vector3 currForce;
    bool interrupted;
    yarp::os::Semaphore semStart;
    yarp::os::Semaphore semDone;
    yarp::os::Mutex guard;
};

