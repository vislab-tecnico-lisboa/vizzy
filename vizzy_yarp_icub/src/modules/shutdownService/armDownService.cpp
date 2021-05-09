#include <iostream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Run.h>
#include <vizzy_msgs/ArmDown.h>
#include <vizzy_msgs/ArmDownReply.h>

using namespace yarp::os;
using namespace yarp::dev;

void do_home_position(vizzy_msgs::ArmDown &msg,Property &my_options,PolyDriver &my_poly_driver,vizzy_msgs::ArmDownReply &reply){
    IPositionControl *pos;
    IEncoders *encs;
    IControlMode2 *ictrl;
    if (msg.arm_down_request==msg.LEFT){
        if (msg.robot_type==msg.REAL){
            my_options.put("robot", "vizzy");
            my_options.put("remote", "/vizzy/left_shoulder_arm");
            my_options.put("local", "/vizzy/left_arm_down_interface");
        }
        else if (msg.robot_type==msg.SIM){
            my_options.put("robot", "vizzySim");
            my_options.put("remote", "/vizzySim/left_shoulder_arm");
            my_options.put("local", "/vizzySim/left_arm_down_interface");
        }
        my_options.put("part", "left_shoulder_arm");
    }
    else if (msg.arm_down_request==msg.RIGHT){
        if (msg.robot_type==msg.REAL){
            my_options.put("robot", "vizzy");
            my_options.put("remote", "/vizzy/right_shoulder_arm");
            my_options.put("local", "/vizzy/right_arm_down_interface");
        }
        else if (msg.robot_type==msg.SIM){
            my_options.put("robot", "vizzySim");
            my_options.put("remote", "/vizzySim/right_shoulder_arm");
            my_options.put("local", "/vizzySim/right_arm_down_interface");
        }
        my_options.put("part", "right_shoulder_arm");
    }
    my_options.put("device", "remote_controlboard");
    
    //Available parts: head torso left_shoulder_arm right_shoulder_arm
    my_poly_driver.open(my_options);
    if (!my_poly_driver.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        reply.arm_down_reply=reply.FAILURE;
    }
    bool ok;
    ok = my_poly_driver.view(pos);
    ok = ok && my_poly_driver.view(encs);
    ok = ok && my_poly_driver.view(ictrl);
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        reply.arm_down_reply=reply.FAILURE;
    }
    int nj=0;
    pos->getAxes(&nj);
    double *positions_down = new double[nj];
    // Setting Control Mode - Position
    for(int i=0;i< nj;i++)
        ictrl->setControlMode(i,VOCAB_CM_POSITION);
    // Setting Motor positions for down arm
    for (int i=0;i< nj;i++)
        positions_down[i] = 0.0;
    pos->positionMove(positions_down);
    bool done=false;
    while(!done) {
        pos->checkMotionDone(&done);
        if (msg.robot_type==msg.SIM)
            Time::delay(1.0);
        else if (msg.robot_type==msg.REAL)
            Time::delay(0.00001);
    }
    reply.arm_down_reply=reply.SUCCESS;
}


int main(int argc, char * argv[])
{
    Network yarp;
    RpcServer server;
    vizzy_msgs::ArmDown example;
    
    server.promiseType(example.getType());
    if (!server.open("/armDown@/arm_down_server")) {
        fprintf(stderr,"Failed to open port\n");
        return 1;
    }

    while (true) {
        vizzy_msgs::ArmDown msg;
        vizzy_msgs::ArmDownReply reply;
        if (!server.read(msg,true)) continue;
        Property options;
        PolyDriver robotDevice;
        do_home_position(msg,options,robotDevice,reply);
        server.reply(reply);
    }
    return 0;
}
