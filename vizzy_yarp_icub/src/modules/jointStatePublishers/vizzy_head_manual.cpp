#include <iostream>
#include <yarp/os/all.h>
//#include <iomanip>
#include <math.h>
#include "sensor_msgs_JointState.h"
using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;
  yarp::os::Node    *rosNode;
  BufferedPort<Bottle> receiverBuff1Mux1;
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/vizzy_head/mux1/receiver1");
  rosNode = new yarp::os::Node("/vizzy_head_bridge");
  yarp::os::Publisher<sensor_msgs_JointState> outputPort;
  bool outputOk_3 = outputPort.topic("/vizzy_head/joint_states");
  outputPort.setWriteOnly();
  

  //Port outputPort;
  //outputPort.setWriteOnly();
  //bool outputOk = outputPort.open("/vizzy_head/joint_states@/yarp/vizzy_head");

  

  if(!outputOk_3)
  {
    printf("outputOk_3 failed to open\n");
    return -1;
  }


  bool connectSuccess = false;
  while(!connectSuccess)
{
    printf("\nTrying to connect to /iKinGazeCtrl/x:o ... ");
    connectSuccess = yarp.connect("/vizzy/head/state:o", receiverBuff1Mux1.getName());
    yarp::os::Time::delay(1);
  }

  int counter = 0;

  while(true){
    Bottle* reading1Mux1;
    reading1Mux1 = receiverBuff1Mux1.read(false);

    if (reading1Mux1 != NULL){
	sensor_msgs_JointState & out = outputPort.prepare();
	int bottleSize;
	bottleSize = reading1Mux1->size();
	out.name.resize(bottleSize);
	out.position.resize(bottleSize);
	out.velocity.resize(bottleSize);
	out.effort.resize(bottleSize);
    for(int i = 0; i < reading1Mux1->size(); i++) {
       out.position[i] = reading1Mux1->get(i).asDouble() / (180/3.1415926);
    }
    out.name[0]= "neck_pan_joint";
    out.name[1]="neck_tilt_joint";
    out.name[2]="eyes_tilt_joint";
    out.name[3]="version_joint";
    out.name[4]="vergence_joint";
    /* DO SOME COMPUTATION HERE */
    double timestamp = (double) Time::now();
    out.header.frame_id="";
    out.header.stamp.sec=(int)timestamp;

    double dummy;
    double frac=modf(timestamp, &dummy);
    out.header.stamp.nsec=(int)round(frac*pow(10,9));
    

    outputPort.write();
    yarp::os::Time::delay(0.03);
  }
  }
  return 0;
}
