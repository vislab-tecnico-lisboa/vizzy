#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>

#include <string>
#include <math.h>
using namespace yarp::dev;
using namespace yarp::os;

int jnts = 0;  // joint number

int main(int argc, char *argv[])
{
    Network yarp;

    Property options;
    options.put("robot", "vizzy");//Needs to be read from a config file
    options.put("device", "remote_controlboard");
    options.put("local", "/vizzy/gaze_pos_interface");
    options.put("remote", "/vizzySim/head");
    //Available parts: head torso left_shoulder_arm right_shoulder_arm
    options.put("part", "head");
    IPositionControl *ipos=0;
    IPositionControl2 *ipos2=0;
    IPositionDirect  *iposDir=0;
    IControlMode2    *imode=0;
    IEncoders *enc=0;
    PolyDriver dd(options);
    bool ok;
    ok = dd.view(ipos);
    ok &= dd.view(ipos2);
    ok &= dd.view(enc);
    ok &= dd.view(iposDir);
    ok &= dd.view(imode);

    if (!ok) {
        yError("Problems acquiring mandatory interfaces, quitting\n");
        return 1;
    }
    BufferedPort<Bottle>  receiverGazeAngles;
    bool receiverOk = receiverGazeAngles.open("/gaze_angles");
    if(!receiverOk)
    {
      printf("/gaze_angles failed to open\n");
      return -1;
    }
    bool connectSuccess = false;
    while(!connectSuccess)
    {
      printf("\nTrying to connect to /iKinGazeCtrl/xd:i ... ");
      connectSuccess = yarp.connect("/iKinGazeCtrl/q:o",receiverGazeAngles.getName());
      yarp::os::Time::delay(1);
    }
    double head_speeds[5] = {15.0,15.0,15.0,10.0,10.0};
    double position_neck[3] = {0.0,0.0,0.0};
    double position_eyes[2] = {0.0,0.0};
    double positions[5] = {0.0,0.0,0.0,0.0,0.0};
    double position_two[8] = {5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0};
    double position_three[8] = {10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0};
    int joint_indexes[2] = {3,4};
    int joint_indexes_neck[3] = {0,1,2};

    iposDir->getAxes(&jnts);
    ipos2->getAxes(&jnts);

    for (size_t j=0; j<5; j++){
        if (j<3)
        imode->setControlMode(joint_indexes_neck[j],VOCAB_CM_POSITION_DIRECT);
        else
            imode->setControlMode(joint_indexes[j-3],VOCAB_CM_MIXED);
    }
    while(true){
      Bottle *reading1Mux1;
      reading1Mux1 = receiverGazeAngles.read(false);
      if (reading1Mux1 != NULL){
          for(int i = 1; i < reading1Mux1->size(); i++) {
              if (i<4){
                  position_neck[i-1]=reading1Mux1->get(i).asDouble();
              }
              else{
                  position_eyes[i-4]=reading1Mux1->get(i).asDouble();
              }
          }
          iposDir->setPositions(3,(const int*)&joint_indexes_neck,position_neck);
          ipos2->positionMove(2,(const int*)&joint_indexes,position_eyes);
      }
    }


  return 0;
}
