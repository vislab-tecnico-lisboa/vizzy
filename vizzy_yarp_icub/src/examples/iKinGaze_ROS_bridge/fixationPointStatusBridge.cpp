#include <iostream>
#include <yarp/os/all.h>
#include <math.h>

using namespace yarp::os;

int main(int argc, char *argv[]) {
  Network yarp;

  BufferedPort<Bottle> receiverBuff1Mux1;
  bool receiver1Mux1Ok = receiverBuff1Mux1.open("/fixationPointStatusBridge/mux1/receiver1");

  Port outputPort;
  outputPort.promiseType(Type::byNameOnWire("geometry_msgs/PointStamped"));
  outputPort.setWriteOnly();
  bool outputOk = outputPort.open("/fixation_point@/yarp/fixationPointStatusBridge");

  bool allConnected = false;
  std::vector<std::vector<bool> > portsConnected;
  portsConnected.resize(1);
  int totalConnections=0;
    portsConnected[0].resize(1);
    portsConnected[0].assign(1,false);
    totalConnections += 1;

  while (!allConnected){
    int connectedNumber=totalConnections;
    if (!portsConnected[0][1]){
      portsConnected[0][1] = yarp.connect("/iKinGazeCtrl/x:o", receiverBuff1Mux1.getName());
      if (!portsConnected[0][1])
        connectedNumber--;
    }    if (connectedNumber == totalConnections)
      allConnected = true;
    std::cout << ".\n";
    Time::delay(1);
  }  int counter = 0;

  while(true){
    Bottle* reading1Mux1 = receiverBuff1Mux1.read();

    Bottle mux1;

    for(int i = 0; i < reading1Mux1->size(); i++) {
      mux1.add(reading1Mux1->get(i));
    }

    for(int i = 0; i < mux1.size(); i++) {
      break;
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

    list_1.add("base_link");

    Bottle& list_2 = message.addList();
    for(int i = 0; i < mux1.size(); i++) {
      list_2.add(mux1.get(i));
    }

    /* DO SOME COMPUTATION HERE */

    outputPort.write(message);
    counter++;
    Time::delay(0.016666666666666666);
  }

  return 0;
}