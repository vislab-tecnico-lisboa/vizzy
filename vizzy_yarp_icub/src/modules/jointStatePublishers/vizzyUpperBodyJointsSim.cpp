#include <iostream>
#include <yarp/os/all.h>
#include <math.h>

using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;

  BufferedPort<Bottle> receiverBuff1Mux1;
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/vizzyUpperBodyJointsSim/mux1/receiver1");
  BufferedPort<Bottle> receiverBuff2Mux1;
  bool receiver2Mux1Ok = receiverBuff2Mux1.open("/vizzyUpperBodyJointsSim/mux1/receiver2");
  BufferedPort<Bottle> receiverBuff3Mux1;
  bool receiver3Mux1Ok = receiverBuff3Mux1.open("/vizzyUpperBodyJointsSim/mux1/receiver3");
  BufferedPort<Bottle> receiverBuff4Mux1;
  bool receiver4Mux1Ok = receiverBuff4Mux1.open("/vizzyUpperBodyJointsSim/mux1/receiver4");

  Port outputPort;
  outputPort.promiseType(Type::byNameOnWire("sensor_msgs/JointState"));
  outputPort.setWriteOnly();
  bool outputOk = outputPort.open("/vizzy_upper_body/joint_states@/yarp/vizzyUpperBodyJointsSim");

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
      portsConnected[0][1] = yarp.connect("/vizzySim/torso/state:o", receiverBuff1Mux1.getName());
      if (!portsConnected[0][1])
        connectedNumber--;
    }    if (!portsConnected[0][2]){
      portsConnected[0][2] = yarp.connect("/vizzySim/head/state:o", receiverBuff2Mux1.getName());
      if (!portsConnected[0][2])
        connectedNumber--;
    }    if (!portsConnected[0][3]){
      portsConnected[0][3] = yarp.connect("/vizzySim/left_shoulder_arm/state:o", receiverBuff3Mux1.getName());
      if (!portsConnected[0][3])
        connectedNumber--;
    }    if (!portsConnected[0][4]){
      portsConnected[0][4] = yarp.connect("/vizzySim/right_shoulder_arm/state:o", receiverBuff4Mux1.getName());
      if (!portsConnected[0][4])
        connectedNumber--;
    }    if (connectedNumber == totalConnections)
      allConnected = true;
    std::cout << ".\n";
    Time::delay(1);
  }  int counter = 0;

  while(true){
    Bottle* reading1Mux1 = receiverBuff1Mux1.read();
    Bottle* reading2Mux1 = receiverBuff2Mux1.read();
    Bottle* reading3Mux1 = receiverBuff3Mux1.read();
    Bottle* reading4Mux1 = receiverBuff4Mux1.read();

    Bottle mux1;

    for(int i = 0; i < reading1Mux1->size(); i++) {
      mux1.add(reading1Mux1->get(i));
    }
    double left_eye = (reading2Mux1->get(3).asDouble()+reading2Mux1->get(4).asDouble())/2;
    double right_eye = (reading2Mux1->get(3).asDouble()-reading2Mux1->get(4).asDouble())/2;
    for(int i = 0; i < reading2Mux1->size(); i++) {
      if (i==3)
          mux1.add(left_eye);
      else if (i==4)
          mux1.add(right_eye);
      else
	  mux1.add(reading2Mux1->get(i));
    }
    for(int i = 0; i < reading3Mux1->size(); i++) {
      mux1.add(reading3Mux1->get(i));
    }
    for(int i = 0; i < reading4Mux1->size(); i++) {
      mux1.add(reading4Mux1->get(i));
    }

    for(int i = 0; i < mux1.size(); i++) {
      mux1.get(i) = mux1.get(i).asDouble() / (180/3.1415926);
    }

    /* DO SOME COMPUTATION HERE */

    double timestamp = (double) Time::now();

    Bottle message = Bottle();

    Bottle& list_1 = message.addList();

    list_1.add(counter);

    double dummy;
    double frac=modf(timestamp,&dummy);
    Bottle& list_1_1 = list_1.addList();
    list_1_1.add((int)timestamp);
    list_1_1.add((int)round(frac*pow(10,9)));

    list_1.add("");

    Bottle& list_2 = message.addList();
    list_2.add("waist_joint");
    list_2.add("neck_pan_joint");
    list_2.add("neck_tilt_joint");
    list_2.add("eyes_tilt_joint");
    list_2.add("l_eye_joint");
    list_2.add("r_eye_joint");
    list_2.add("l_shoulder_scapula_joint");
    list_2.add("l_shoulder_flection_joint");
    list_2.add("l_shoulder_abduction_joint");
    list_2.add("l_shoulder_rotation_joint");
    list_2.add("l_elbow_flection_joint");
    list_2.add("l_forearm_pronation_joint");
    list_2.add("l_wrist_abduction_joint");
    list_2.add("l_wrist_flection_joint");
    list_2.add("r_shoulder_scapula_joint");
    list_2.add("r_shoulder_flection_joint");
    list_2.add("r_shoulder_abduction_joint");
    list_2.add("r_shoulder_rotation_joint");
    list_2.add("r_elbow_flection_joint");
    list_2.add("r_forearm_pronation_joint");
    list_2.add("r_wrist_abduction_joint_virtual");
    list_2.add("r_wrist_flection_joint");

    Bottle& list_3 = message.addList();
    for(int i = 0; i < mux1.size(); i++) {
      list_3.add(mux1.get(i));
    }

    Bottle& list_4 = message.addList();

    Bottle& list_5 = message.addList();

    /* DO SOME COMPUTATION HERE */

    outputPort.write(message);
    counter++;
    Time::delay(0.016666666666666666);
  }

  return 0;
}
