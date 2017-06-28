#include <iostream>
#include <yarp/os/all.h>
#include <math.h>

using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;

  BufferedPort<Bottle> receiverBuff1Mux1;
  receiverBuff1Mux1.setReadOnly();
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/fixation_point_goal_ros@/mux1/receiver1");

  Port outputPort;
  outputPort.setWriteOnly();
  bool outputOk = outputPort.open("/writeBuff ");

  std::cout << "Waiting for output..." << std::endl;
  bool connectSuccess = false;
  while(!connectSuccess) {
    connectSuccess = yarp.connect(" /writeBuff", "/iKinGazeCtrl/xd:i");
    Time::delay(1);
    std::cout << ".\n";
  }
  std::cout << "Connection successfuly established." << std::endl;

  int counter = 0;

  while(true){
    Bottle* reading1Mux1 = receiverBuff1Mux1.read(false);

    Bottle mux1;

    for(int i = 0; i < reading1Mux1->size(); i++) {
      mux1.add(reading1Mux1->get(i));
    }

    for(int i = 0; i < mux1.size(); i++) {
      std::cout << "Dummy break. Sorry for my laziness." << std::endl;
      break;
    }

    /* DO SOME COMPUTATION HERE */

    double timestamp = (double) Time::now();

    Bottle message = Bottle();

    for(int i = 0; i < mux1.size(); i++) {
       message.add(mux1.get(i));
    }

    /* DO SOME COMPUTATION HERE */

    outputPort.write(message);
    counter++;
    Time::delay(0.016666666666666666);
  }

  return 0;
}
