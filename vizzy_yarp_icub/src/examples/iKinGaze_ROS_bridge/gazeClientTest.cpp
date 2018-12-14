#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

PolyDriver *drvGazeCtrl;
IGazeControl      *gazeCtrl;

int main(int argc, char *argv[]) {
  Network yarp;

Property optGazeCtrl("(device gazecontrollerclient)");
 std::string name="/vizzy_local";
optGazeCtrl.put("remote","/iKinGazeCtrl");
optGazeCtrl.put("local",(name+"/gaze").c_str());

drvGazeCtrl=new PolyDriver;
if (!drvGazeCtrl->open(optGazeCtrl))
  {
    return 0;
  }
 drvGazeCtrl->view(gazeCtrl);
 double step_size=0.1/5;
 double rate=60.0;
 //double t0 = Time::now();
 // spinning
 Vector homeHead(3);
 int i=0;
 while(true)
   {

     i++;

     double angular_freq=2*3.141592*step_size;
     double time_instant=(double)i;
     double aux=angular_freq*time_instant;
     homeHead[0]=-4.81938;
     homeHead[1]=3*cos(aux);
     homeHead[2]=0.449058;
     gazeCtrl->lookAtFixationPoint(homeHead);
     yarp::os::Time::delay(1/rate);
   }
 return 0;
}
