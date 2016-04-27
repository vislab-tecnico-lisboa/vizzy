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
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/vizzyUpperBodyJoints/mux1/receiver1");
  BufferedPort<Bottle> receiverBuff2Mux1;
  bool receiver2Mux1Ok = receiverBuff2Mux1.open("/vizzyUpperBodyJoints/mux1/receiver2");
  BufferedPort<Bottle> receiverBuff3Mux1;
  bool receiver3Mux1Ok = receiverBuff3Mux1.open("/vizzyUpperBodyJoints/mux1/receiver3");
  BufferedPort<Bottle> receiverBuff4Mux1;
  bool receiver4Mux1Ok = receiverBuff4Mux1.open("/vizzyUpperBodyJoints/mux1/receiver4");

  rosNode = new yarp::os::Node("/vizzy_upper_body_bridge");
  yarp::os::Publisher<sensor_msgs_JointState> outputPort;
  bool outputOk_3 = outputPort.topic("/vizzy_upper_body/joint_states");
  outputPort.setWriteOnly();
  

  //Port outputPort;
  //outputPort.setWriteOnly();
  //bool outputOk = outputPort.open("/vizzy_head/joint_states@/yarp/vizzy_head");

  

  if(!outputOk_3)
  {
    printf("outputOk_3 failed to open\n");
    return -1;
  }


  bool allConnected = false;
  std::vector<std::vector<bool> > portsConnected;
  portsConnected.resize(1);
  int totalConnections=0;
    portsConnected[0].resize(4);
    portsConnected[0].assign(4,false);
    totalConnections += 4;

  while (!allConnected){
    int connectedNumber=totalConnections;
    if (!portsConnected[0][1]){
      portsConnected[0][1] = yarp.connect("/vizzy/torso/state:o", receiverBuff1Mux1.getName());
      if (!portsConnected[0][1])
        connectedNumber--;
    }    if (!portsConnected[0][2]){
      portsConnected[0][2] = yarp.connect("/vizzy/head/state:o", receiverBuff2Mux1.getName());
      if (!portsConnected[0][2])
        connectedNumber--;
    }    if (!portsConnected[0][3]){
      portsConnected[0][3] = yarp.connect("/vizzy/left_shoulder_arm/state:o", receiverBuff3Mux1.getName());
      if (!portsConnected[0][3])
        connectedNumber--;
    }    if (!portsConnected[0][4]){
      portsConnected[0][4] = yarp.connect("/vizzy/right_shoulder_arm/state:o", receiverBuff4Mux1.getName());
      if (!portsConnected[0][4])
        connectedNumber--;
    }    if (connectedNumber == totalConnections)
      allConnected = true;
    std::cout << ".\n";
    Time::delay(1);
  }

  int counter = 0;

  while(true){
    //Bottle* reading1Mux1;
    //reading1Mux1 = receiverBuff1Mux1.read(false);
    Bottle* reading1Mux1 = receiverBuff1Mux1.read();
    Bottle* reading2Mux1 = receiverBuff2Mux1.read();
    Bottle* reading3Mux1 = receiverBuff3Mux1.read();
    Bottle* reading4Mux1 = receiverBuff4Mux1.read();
    int totalBottleSize;
    totalBottleSize = reading1Mux1->size() + reading2Mux1->size() + reading3Mux1->size() + reading4Mux1->size();
    
    if (reading1Mux1 != NULL && reading2Mux1 != NULL){
	sensor_msgs_JointState & out = outputPort.prepare();
	int bottleSize;
	bottleSize = reading1Mux1->size();
	out.name.resize(totalBottleSize);
	out.position.resize(totalBottleSize);
	out.velocity.resize(totalBottleSize);
	out.effort.resize(totalBottleSize);
    int i = 0;
    for(int j=0; j < reading1Mux1->size(); i++,j++) {
       out.position[i] = reading1Mux1->get(j).asDouble() / (180/3.1415926);
    }
    for(int j=0; j < reading2Mux1->size(); i++,j++) {
       out.position[i] = reading2Mux1->get(j).asDouble() / (180/3.1415926);
    }
    for(int j=0; j < reading3Mux1->size(); i++,j++) {
       out.position[i] = reading3Mux1->get(j).asDouble() / (180/3.1415926);
    }
    for(int j=0; j < reading4Mux1->size(); i++,j++) {
       out.position[i] = reading4Mux1->get(j).asDouble() / (180/3.1415926);
    }
    out.name[0] = "waist_joint";
    out.name[1] = "neck_pan_joint";
    out.name[2] = "neck_tilt_joint";
    out.name[3] = "eyes_tilt_joint";
    out.name[4] = "version_joint";
    out.name[5] = "vergence_joint";
    out.name[6] = "l_shoulder_scapula_joint";
    out.name[7] = "l_shoulder_flection_joint";
    out.name[8] = "l_shoulder_abduction_joint";
    out.name[9] = "l_shoulder_rotation_joint";
    out.name[10] = "l_elbow_flection_joint";
    out.name[11] = "l_forearm_pronation_joint";
    out.name[12] = "l_wrist_abduction_joint";
    out.name[13] = "l_wrist_flection_joint";
    out.name[14] = "l_thumb_flection";
    out.name[15] = "l_index_flection";
    out.name[16] = "l_ring_pinky_flection";
    out.name[17] = "r_shoulder_scapula_joint";
    out.name[18] = "r_shoulder_flection_joint";
    out.name[19] = "r_shoulder_abduction_joint";
    out.name[20] = "r_shoulder_rotation_joint";
    out.name[21] = "r_elbow_flection_joint";
    out.name[22] = "r_forearm_pronation_joint";
    out.name[23] = "r_wrist_abduction_joint";
    out.name[24] = "r_wrist_flection_joint";
    out.name[25] = "r_thumb_flection";
    out.name[26] = "r_index_flection";
    out.name[27] = "r_ring_pinky_flection";
    
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
