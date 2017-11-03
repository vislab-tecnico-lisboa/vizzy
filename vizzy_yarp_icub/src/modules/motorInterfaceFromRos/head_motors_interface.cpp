#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>

#include <string>
#include "Float64.h"
using namespace yarp::dev;
using namespace yarp::os;

int jnts = 0;  // joint number
IPositionControl *ipos=0;
//Node *node=0;
yarp::os::Subscriber<Float64> subscriber;
yarp::os::Subscriber<Float64> subscriber_two;
Semaphore controller_mutex;

class Thread1 : public Thread {
public:
//    Thread1():Thread(){}
    Thread1(yarp::os::Subscriber<Float64> *my_topic__,int joint){
	setSubscriber(my_topic__);
	joint_index = joint;
    }
    virtual bool threadInit()
    {
        //printf("Starting thread1\n");
	Time::delay(0.01);
        return true;
    }
    virtual void setSubscriber(yarp::os::Subscriber<Float64> *my_topic__){
	my_topic = my_topic__;
    }
    //called by start after threadInit, s is true iff the thread started
    //successfully
    virtual void run() 
    {
	while (!isStopping()) {
        //printf("Hello, from thread1\n");
	Float64 *data;
        data = my_topic->read();
	if (data != NULL){
        //std::cout << "Received:" << data->data << " " << std::endl;
	controller_mutex.wait();
	ipos->positionMove(joint_index, data->data*180.0/3.141592);
	controller_mutex.post();
	bool motionDone=false;
	while (motionDone==false)
		ipos->checkMotionDone(joint_index,&motionDone);
	//std::cout << "Motion to :" << data->data*180.0/3.141592 << " sent! " << std::endl;
	}
	}
    }
    virtual void threadRelease()
    {
        printf("Goodbye from thread1\n");
    }
private:
    yarp::os::Subscriber<Float64> *my_topic;
    int joint_index;
};


int main(int argc, char *argv[]) 
{
    Network yarp;

    Property options;
    options.put("robot", "vizzySim");//Needs to be read from a config file
    options.put("device", "remote_controlboard");
    options.put("local", "/vizzy/head_pos_interface");
    options.put("remote", "/vizzySim/head");
    //Available parts: head torso left_shoulder_arm right_shoulder_arm
    options.put("part", "head");
    IPositionControl2 *ipos2=0;
    IPositionDirect  *iposDir=0;
    IVelocityControl2 *vel=0;
    IEncoders *enc=0;
    IPidControl *pid=0;
    IAmplifierControl *amp=0;
    IControlLimits *lim=0;
    IControlLimits2 *lim2 = 0;
    IControlMode2 *iMode2=0;
    IMotor *imot=0;
    ITorqueControl *itorque=0;
    //IOpenLoopControl *iopenloop=0;
    IImpedanceControl *iimp=0;
    IInteractionMode *iInteract=0;
    IMotorEncoders *iMotEnc=0;
    IAxisInfo *iInfo = 0;
    PolyDriver dd(options);
    bool ok;
    ok = dd.view(ipos);
    /*ok &= dd.view(ipos2);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(lim);
    ok &= dd.view(lim2);
//    ok &= dd.view(icm);
    ok &= dd.view(itorque);
    ok &= dd.view(iopenloop);
    ok &= dd.view(iimp);
    ok &= dd.view(iposDir);
    ok &= dd.view(iMode2);
    ok &= dd.view(iInteract);*/

    if (!ok) {
        yError("Problems acquiring mandatory interfaces, quitting\n");
        return 1;
    }
    double head_speeds[5] = {30.0,30.0,30.0,30.0,30.0};
    ipos->setRefSpeeds(head_speeds);
    /* creates a node called /yarp/listener */
    Node node("/vizzy/motor_interface");
    bool connected=false;
    bool connected_one=false;
    while (!connected){
	connected = subscriber.topic("/vizzy/neck_pan_joint/cmd");
	connected_one = subscriber_two.topic("/vizzy/neck_tilt_joint/cmd");
        if (!connected){
	    std::cout << "Failed to subscriber to /vizzy/neck_pan_joint/cmd" << std::endl;
            Time::delay(1);
        }
	else if (!connected_one){
	    std::cout << "Failed to subscriber to /vizzy/neck_tilt_joint/cmd" << std::endl;
            Time::delay(1);
        }
	else
	    std::cout << "Subscribers initialized" << std::endl;
    }
    Thread1 t1(&subscriber,0);
    Thread1 t2(&subscriber_two,1);
    ok=t1.start();
    ok &= t2.start();
        if (!ok)
        {
            printf("One of the threads failed to initialize, returning\n");
            return -1;
        }
    return 0;
}
