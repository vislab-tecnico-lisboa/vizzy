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
    options.put("local", "/vizzy/left_arm_pos_interface");
    options.put("remote", "/vizzy/left_shoulder_arm");
    //Available parts: head torso left_shoulder_arm right_shoulder_arm
    options.put("part", "left_shoulder_arm");
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
    //double left_arm_speeds[8] = {10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0};
    double position_one[8] = {10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0};
    double position_two[8] = {20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0};
    double position_three[8] = {30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0};
    int joint_indexes[8] = {0,1,2,3,4,5,6,7};
    //iposDir->setRefSpeeds(left_arm_speeds);
    //imode->setControlMode(j,VOCAB_CM_POSITION);
    iposDir->getAxes(&jnts);
    for (size_t j=0; j<8; j++){
        imode->setControlMode(joint_indexes[j],VOCAB_CM_POSITION_DIRECT);
	//iposDir->setPosition(joint_indexes[j],position_one[j]);
    }
    iposDir->setPositions(jnts,(const int*)&joint_indexes,position_one);
    /*std::cout << "Position one sent to position control 2! " << std::endl;
    ipos2->positionMove(jnts,(const int*)&joint_indexes,position_two);
    std::cout << "Position two sent to position control 2! " << std::endl;
    ipos2->positionMove(jnts,(const int*)&joint_indexes,position_three);
    std::cout << "Position three sent to position control 2! " << std::endl;
    iposDir->setPositions(jnts,(const int*)&joint_indexes,position_one);
    std::cout << "Position one sent to position direct! " << std::endl;
    iposDir->setPositions(jnts,(const int*)&joint_indexes,position_three);
    std::cout << "Position three sent to position direct! " << std::endl;*/
  return 0;
}
