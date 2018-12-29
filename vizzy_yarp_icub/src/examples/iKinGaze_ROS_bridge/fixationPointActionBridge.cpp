#include <iostream>
#include <yarp/os/all.h>
#include <math.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include "geometry_msgs_Point.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

int main(int argc, char *argv[])
{
  Network yarp;

  PolyDriver *drvGazeCtrl;
  IGazeControl *gazeCtrl;

  Node node("/vizzy/left_arm_motor_interface");
  yarp::os::Subscriber<geometry_msgs_Point> subscriber_ros_goal;

  bool allConnected = false;
  bool portsConnected;
  while (!allConnected)
  {
    portsConnected = subscriber_ros_goal.topic("/fixation_point_out");
    if (portsConnected)
      allConnected = true;
    else
    {
      std::cout << ".\n";
      Time::delay(1);
    }
  }
  Property optGazeCtrl("(device gazecontrollerclient)");
  std::string name = "/vizzy_local";
  optGazeCtrl.put("remote", "/iKinGazeCtrl");
  optGazeCtrl.put("local", (name + "/gaze").c_str());

  drvGazeCtrl = new PolyDriver;
  Vector posHead(3);
  if (!drvGazeCtrl->open(optGazeCtrl))
  {
    return 0;
  }
  drvGazeCtrl->view(gazeCtrl);

  geometry_msgs_Point *data;
  while (true)
  {

    data = subscriber_ros_goal.read();
    posHead[0] = data->x;
    posHead[1] = data->y;
    posHead[3] = data->z;
    gazeCtrl->lookAtFixationPoint(posHead);
  }

  return 0;
}
