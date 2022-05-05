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
#include <yarp/dev/all.h>


using namespace yarp::dev;
using namespace yarp::sig;

class RunControl : public yarp::os::Thread
{
public:

    RunControl():semStart(0) { }
    virtual ~RunControl();
    virtual bool threadInit();
    RunControl(yarp::os::Subscriber<TactSensorArray> *my_topic__, IEncoders *encs__,
        IPositionControl *pos__);
    virtual void setSubscriber(yarp::os::Subscriber<TactSensorArray> *my_topic__);
    virtual void run();
    void get_force(int index,yarp::sig::Vector& force);
    virtual void threadRelease();
    void EnableControl();
    void DisableControl();
private:
    yarp::os::Subscriber<TactSensorArray> *my_topic;
    yarp::sig::Vector* image;
    std::vector<TactSensor>* array;
    bool interrupted;
    yarp::os::Semaphore semStart;
    yarp::os::Semaphore semDone;
    yarp::os::Mutex guard;
    yarp::sig::Vector encoders;
    bool controlActive;
    
    IPositionControl *pos;
    IEncoders *encs;

    double finger_set[3];       // new force setpoints
    double finger_force[3];         // current force values
    double joint_inc[3];
    double inc_max;                           // max joint increment
    double joint_max[3];                        // max joint value for fingers (min is 0)
    double sensor_set[11];
    double force_error;                      // accepeted force error [N]
    bool make_control;
    bool naiveSumControl;
	
    Vector sensor_force;
};