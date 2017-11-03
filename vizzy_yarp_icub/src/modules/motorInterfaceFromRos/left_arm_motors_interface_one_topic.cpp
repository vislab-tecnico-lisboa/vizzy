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
#include "motorsArray.h"
using namespace yarp::dev;
using namespace yarp::os;

int jnts = 0;  // joint number

int main(int argc, char *argv[]) 
{
    Network yarp;

    yarp::os::Subscriber<motorsArray> subscriber_left_arm;

    Property options;
    options.put("robot", "vizzySim");//Needs to be read from a config file
    options.put("device", "remote_controlboard");
    options.put("local", "/vizzy/left_arm_pos_interface");
    options.put("remote", "/vizzySim/left_shoulder_arm");
    //Available parts: head torso left_shoulder_arm right_shoulder_arm
    options.put("part", "left_shoulder_arm");
    IPositionControl *ipos=0;
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
    ipos->getAxes(&jnts);
    printf("Working with %d axes\n", jnts);
    double *tmp = new double[jnts];
    /* creates a node called /yarp/listener */
    Node node("/vizzy/left_arm_motor_interface");


  bool allConnected = false;
  bool portsConnected;
  while (!allConnected){
	portsConnected = subscriber_left_arm.topic("/vizzy/l_shoulder_arm");
	if (portsConnected)
		allConnected = true;
	else {
	    std::cout << ".\n";
    	    Time::delay(1);
	}
  }
  while (true) {
	//printf("Hello, from thread1\n");
	std::cout << "Hello from left arm !!" << std::endl;
	motorsArray *data;
	//data->data.resize(8);
        data = subscriber_left_arm.read(); //data->data*180.0/3.141592
	if (data != NULL){
	for (int my_i=0;my_i<jnts;my_i++){
		tmp[my_i] = data->data[my_i]*180.0/3.141592;
		std::cout << "Motion to :" << data->data[my_i]*180.0/3.141592 << "sent!" << std::endl;
	}
	ipos->positionMove(tmp);
	bool motionDone=false;
	while (motionDone==false)
		ipos->checkMotionDone(&motionDone);
	//std::cout << "Motion to :" << data->data*180.0/3.141592 << " sent! " << std::endl;
	}
	std::cout << "..." << std::endl;
  }
  return 0;
}
