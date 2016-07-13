#include <iostream>
#include <numeric>
#include <yarp/os/all.h>
#include <math.h>

using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;

  BufferedPort<Bottle> receiverBuff1Mux1;
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/vizzyUpperBodyJoints/mux1/receiver1");
  BufferedPort<Bottle> receiverBuff2Mux1;
  bool receiver2Mux1Ok = receiverBuff2Mux1.open("/vizzyUpperBodyJoints/mux1/receiver2");
  BufferedPort<Bottle> receiverBuff3Mux1;
  bool receiver3Mux1Ok = receiverBuff3Mux1.open("/vizzyUpperBodyJoints/mux1/receiver3");
  BufferedPort<Bottle> receiverBuff4Mux1;
  bool receiver4Mux1Ok = receiverBuff4Mux1.open("/vizzyUpperBodyJoints/mux1/receiver4");

  Port outputPort;
  outputPort.promiseType(Type::byNameOnWire("JointState"));
  outputPort.setWriteOnly();
  bool outputOk = outputPort.open("/vizzy_upper_body/joint_states@/yarp/vizzyUpperBodyJoints");

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
  }  int counter = 0;
  Bottle mux1_previous;
  double timestamp_previous,timestamp;
  // Filter that smooths the velocity values
  // From the matlab command sgolay(
  std::vector<double > golayFilter;
  golayFilter.resize(7);
  golayFilter[0]=-0.1071;
  golayFilter[1]=-0.0714;
  golayFilter[2]=-0.0357;
  golayFilter[3]=0.0;
  golayFilter[4]=0.0357;
  golayFilter[5]=0.0714;
  golayFilter[6]=0.1071;
  std::vector<std::vector<double> > jointsBuffer;
  jointsBuffer.resize(28);

  while(true){
    Bottle* reading1Mux1 = receiverBuff1Mux1.read(false);
    Bottle* reading2Mux1 = receiverBuff2Mux1.read(false);
    Bottle* reading3Mux1 = receiverBuff3Mux1.read(false);
    Bottle* reading4Mux1 = receiverBuff4Mux1.read(false);

    Bottle mux1;
    if (reading1Mux1 != NULL && reading2Mux1 != NULL && reading3Mux1 != NULL && reading4Mux1 != NULL){ 
    for(int i = 0; i < reading1Mux1->size(); i++) {
      mux1.add(reading1Mux1->get(i));
    }
    //std::cout << "Version: " << reading2Mux1->get(3).asDouble() << std::endl;
    //std::cout << "Vergence: " << reading2Mux1->get(4).asDouble() << std::endl;
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

    timestamp = (double) Time::now();
  
    Bottle message = Bottle();

    Bottle& list_1 = message.addList();

    list_1.add(counter);

    double dummy;
    double frac=modf(timestamp,&dummy);

    if (counter==0){
	timestamp_previous = timestamp;
	mux1_previous.copy(mux1);
	counter ++;
	continue;
    }
    Bottle& list_1_1 = list_1.addList();
    //list_1_1.add(counter);
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
    list_2.add("l_thumb_flection");
    list_2.add("l_index_flection");
    list_2.add("l_ring_pinky_flection");
    list_2.add("r_shoulder_scapula_joint");
    list_2.add("r_shoulder_flection_joint");
    list_2.add("r_shoulder_abduction_joint");
    list_2.add("r_shoulder_rotation_joint");
    list_2.add("r_elbow_flection_joint");
    list_2.add("r_forearm_pronation_joint");
    list_2.add("r_wrist_abduction_joint");
    list_2.add("r_wrist_flection_joint");
    list_2.add("r_thumb_flection");
    list_2.add("r_index_flection");
    list_2.add("r_ring_pinky_flection");

    Bottle& list_3 = message.addList();
    for(int i = 0; i < mux1.size(); i++) {
      list_3.add(mux1.get(i));
      if (jointsBuffer[i].size()<7){
	 jointsBuffer[i].push_back(mux1.get(i).asDouble());
      }
      else{
	jointsBuffer[i].push_back(mux1.get(i).asDouble());
	jointsBuffer[i].erase (jointsBuffer[i].begin());
      }
    }

    Bottle& list_4 = message.addList();
    for(int i = 0; i < mux1.size(); i++) {
      if (jointsBuffer[i].size()<7)
        list_4.add((mux1.get(i).asDouble()-mux1_previous.get(i).asDouble())/(timestamp-timestamp_previous));
      else{
        double delta_t = (timestamp-timestamp_previous);
        double delta_ang = std::inner_product( golayFilter.begin(), golayFilter.end(), jointsBuffer[i].begin(), 0.0 ); 
        double my_vel = delta_ang/delta_t;
	list_4.add(my_vel);
      }
    }

    Bottle& list_5 = message.addList();
    for(int i = 0; i < mux1.size(); i++) {
      list_5.add(mux1.get(i));
    }
    /* DO SOME COMPUTATION HERE */

    outputPort.write(message);
    mux1_previous.copy(mux1);
    }
    counter++;
    timestamp_previous = timestamp;
    Time::delay(0.0166666);
  }

  return 0;
}
