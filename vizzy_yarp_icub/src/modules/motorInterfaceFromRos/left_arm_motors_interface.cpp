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
//yarp::os::Subscriber<Float64> subscriber;
//yarp::os::Subscriber<Float64> subscriber_two;
//std::vector<yarp::os::Subscriber<Float64> > subscribers_left_arm;//(8);//,yarp::os::Subscriber<Float64>());
yarp::os::Subscriber<Float64> subscribers_left_arm[8];
Semaphore controller_mutex;

class Thread1 : public Thread {
public:
//    Thread1():Thread(){}
    Thread1(){}
    Thread1(yarp::os::Subscriber<Float64> *my_topic__,int joint){
	setSubscriber_and_joint(my_topic__,joint);
    }
    virtual bool threadInit()
    {
        //printf("Starting thread1\n");
	Time::delay(0.01);
        return true;
    }
    virtual void setSubscriber_and_joint(yarp::os::Subscriber<Float64> *my_topic__,int joint){
	my_topic = my_topic__;
	joint_index = joint;
    }
    //called by start after threadInit, s is true iff the thread started
    //successfully
    virtual void run() 
    {
	while (!isStopping()) {
        //printf("Hello, from thread1\n");
	std::cout << "Hello from left arm thread:" << joint_index << " " << std::endl;
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
    options.put("local", "/vizzy/left_arm_pos_interface");
    options.put("remote", "/vizzySim/left_shoulder_arm");
    //Available parts: head torso left_shoulder_arm right_shoulder_arm
    options.put("part", "left_shoulder_arm");
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
    double head_speeds[8] = {30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0};
    ipos->setRefSpeeds(head_speeds);
    /* creates a node called /yarp/listener */
    Node node("/vizzy/left_arm_motor_interface");


  bool allConnected = false;
  std::vector<bool > portsConnected;
  portsConnected.resize(8);
  int totalConnections=0;
  portsConnected.assign(8,false);
  totalConnections += 8;

  //std::vector<yarp::os::Subscriber<Float64> > subscribers_left_arm = std::vector<yarp::os::Subscriber<Float64> >(8,yarp::os::Subscriber<Float64>());
  //subscribers_left_arm.resize(8);
  /*for (int i=1;i<8;i++){
	yarp::os::Subscriber<Float64> my_subs = yarp::os::Subscriber<Float64>();
	subscribers_left_arm.push_back(my_subs);
  }*/
  while (!allConnected){
    int connectedNumber=totalConnections;
    if (!portsConnected[0]){
      portsConnected[0] = subscribers_left_arm[0].topic("/vizzy/l_shoulder_scapula_joint/cmd");
      if (!portsConnected[0])
        connectedNumber--;
    }    if (!portsConnected[1]){
      portsConnected[1] = subscribers_left_arm[1].topic("/vizzy/l_shoulder_flection_joint/cmd");
      if (!portsConnected[1])
        connectedNumber--;
    }    if (!portsConnected[2]){
      portsConnected[2] = subscribers_left_arm[2].topic("/vizzy/l_shoulder_abduction_joint/cmd");
      if (!portsConnected[2])
        connectedNumber--;
    }    if (!portsConnected[3]){
      portsConnected[3] = subscribers_left_arm[3].topic("/vizzy/l_shoulder_rotation_joint/cmd");
      if (!portsConnected[3])
        connectedNumber--;
    }    if (!portsConnected[4]){
      portsConnected[4] = subscribers_left_arm[4].topic("/vizzy/l_elbow_flection_joint/cmd");
      if (!portsConnected[4])
        connectedNumber--;
    }    if (!portsConnected[5]){
      portsConnected[5] = subscribers_left_arm[5].topic("/vizzy/l_forearm_pronation_joint/cmd");
      if (!portsConnected[5])
        connectedNumber--;
    }    if (!portsConnected[6]){
      portsConnected[6] = subscribers_left_arm[6].topic("/vizzy/l_wrist_abduction_joint/cmd");
      if (!portsConnected[6])
        connectedNumber--;
    }    if (!portsConnected[7]){
      portsConnected[7] = subscribers_left_arm[7].topic("/vizzy/l_wrist_flection_joint/cmd");
      if (!portsConnected[7])
        connectedNumber--;
    }if (connectedNumber == 0)
      allConnected = true;
    std::cout << ".\n";
    Time::delay(1);
  }
  Thread1 vector_of_threads[8];
    for (int i=0;i<8;i++){
	//vector_of_threads.push_back(Thread1(&subscribers_left_arm[i],i));
	vector_of_threads[i].setSubscriber_and_joint(&subscribers_left_arm[i],i);
    	ok&=vector_of_threads[i].start();
    }
        if (!ok)
        {
            printf("One of the threads failed to initialize, returning\n");
            return -1;
        }
    return 0;
}
