#include <iostream>
#include <yarp/os/all.h>
#include <iomanip>
#include <math.h>
using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;

  BufferedPort<Bottle> receiverBuff1Mux1;
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/vizzy_head/mux1/receiver1");

  Port outputPort;
  outputPort.setWriteOnly();
  bool outputOk = outputPort.open("/vizzy_head/joint_states@/yarp/vizzy_head");

  yarp.connect("/vizzy/head/state:o", receiverBuff1Mux1.getName());

  std::cout << "Waiting for output..." << std::endl;
  while(outputPort.getOutputCount() == 0) {
    Time::delay(1);
    std::cout << ".\n";
  }
  std::cout << "Connection successfuly established." << std::endl;

  int counter = 0;

  while(true){
    Bottle* reading1Mux1 = receiverBuff1Mux1.read();

    Bottle mux1;

    for(int i = 0; i < reading1Mux1->size(); i++) {
      mux1.add(reading1Mux1->get(i));
    }

    for(int i = 0; i < mux1.size(); i++) {
      mux1.get(i) = mux1.get(i).asDouble() / (180/3.1415926);
    }
    Bottle message = Bottle();
    /* DO SOME COMPUTATION HERE */

    double timestamp = (double) Time::now();
    //std::cout << std::fixed;
    //std::cout << std::setprecision(5);
    //std::cout << "timestamp:" << timestamp << std::endl;

	

    double dummy;
    double frac=modf(timestamp, &dummy);


    Bottle& list_1 = message.addList();

    list_1.add(counter);

    Bottle& list_1_1 = list_1.addList();
    list_1_1.add((int)timestamp);
    list_1_1.add((int)round(frac*pow(10,9)));

    list_1.add("0");

    Bottle& list_2 = message.addList();
    list_2.add("neck_pan_joint");
    list_2.add("neck_tilt_joint");
    list_2.add("eyes_tilt_joint");
    list_2.add("version_joint");
    list_2.add("vergence_joint");

    Bottle& list_3 = message.addList();
    for(int i = 0; i < mux1.size(); i++) {
      list_3.add(mux1.get(i));
    }

    Bottle& list_4 = message.addList();

    Bottle& list_5 = message.addList();

    /* DO SOME COMPUTATION HERE */

    outputPort.write(message);
    counter++;
    Time::delay(0.01);
  }

  return 0;
}
