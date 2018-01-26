#include <vizzy_tactile_TactSensor.h>
#include <vizzy_tactile_TactSensorArray.h>
#include <iostream>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Node.h>
using namespace std;
int main()
{
   yarp::os::Network network;
   cout<<"Starting receiver\n";
   //yarp::os::BufferedPort<vizzy_tactile_TactSensorArray> port;
   yarp::os::Subscriber<vizzy_tactile_TactSensorArray> port;
   /*port.setReadOnly();
   if (!port.open("/receiver"))
   {
       cerr<<"Error opening port, check your yarp network\n";
       return -1;
   }
bool connectSuccess = false;
  while(!connectSuccess) {
    connectSuccess = network.connect("/tactileForceField@/tactComputerRightHand", "/receiver");
    yarp::os::Time::delay(1);
    std::cout << ".\n";
  }*/
     yarp::os::Node node("/yarp/listener");
  if (!port.topic("/tactileForceField")) {
        cerr<< "Failed to subscriber to /tactileForceField\n";
        return -1;
    }

  std::cout << "Connection successfuly established." << std::endl;
   
   int count=0;
   while(true)
   {
       vizzy_tactile_TactSensorArray reading1Mux;
       port.read(reading1Mux);
       //vizzy_tactile_TactSensorArray d;
       geometry_msgs_Vector3 currForce;
       //access d
       //d = reading1Mux->sensorArray;
       std::vector<vizzy_tactile_TactSensor> array = reading1Mux.sensorArray;
       currForce = array[0].force;
       std::cout << "Fx : " << currForce.x << " Fy : " << currForce.y << " Fz : " << currForce.z << std::endl;
       
       count++;
   }
   return 0;
}
