#include <iostream>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <yarp/os/Run.h>
#include <vizzy_msgs/MotorsShutdown.h>
#include <vizzy_msgs/MotorsShutdownReply.h>

using namespace yarp::os;
using namespace yarp::dev;

int main(int argc, char * argv[])
{
    Network yarp;
    RpcServer server;
    vizzy_msgs::MotorsShutdown example;
    yarp::os::Run tst;
    
    Property options_left,options_right;
    options_left.put("robot", "vizzy");
    options_left.put("remote", "/vizzy/left_shoulder_arm");
    options_left.put("local", "/vizzy/left_arm_idle_interface");
    options_left.put("part", "left_shoulder_arm");
    options_left.put("device", "remote_controlboard");

    options_right.put("robot", "vizzy");
    options_right.put("remote", "/vizzy/right_shoulder_arm");
    options_right.put("local", "/vizzy/right_arm_idle_interface");
    options_right.put("part", "right_shoulder_arm");
    options_right.put("device", "remote_controlboard");

    PolyDriver right_arm,left_arm;
    right_arm.open(options_right);
    left_arm.open(options_left);
    if (!right_arm.isValid() || !left_arm.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }
    IControlMode2 *ictrl_left_arm,*ictrl_right_arm;
    IPositionControl *pos_left_arm,*pos_right_arm;
    IEncoders *encs_left_arm,*encs_right_arm;
    bool ok;
    ok = right_arm.view(pos_right_arm);
    ok = ok && right_arm.view(encs_right_arm);
    ok = ok && right_arm.view(ictrl_right_arm);
    ok = ok && left_arm.view(pos_left_arm);
    ok = ok && left_arm.view(encs_left_arm);
    ok = ok && left_arm.view(ictrl_left_arm);
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    server.promiseType(example.getType());
    if (!server.open("/armMotorsIdle@/arm_motors_idle_server")) {
        fprintf(stderr,"Failed to open port\n");
        return 1;
    }

    while (true) {
        vizzy_msgs::MotorsShutdown msg;
        vizzy_msgs::MotorsShutdownReply reply;
        if (!server.read(msg,true)) continue;
        reply.shutdown_reply = reply.SUCCESS;
        if (msg.shutdown_request==msg.SHUTDOWN){
            int nj_left=0;
            pos_left_arm->getAxes(&nj_left);
            // Setting Control Mode - Position
            for(int i=0;i< 8;i++){
                bool my_res = ictrl_left_arm->setControlMode(i,VOCAB_CM_IDLE);
                if (!my_res){
                    reply.shutdown_reply = reply.FAILURE;
                    break;
                }
            }
	        for (size_t n=0;n<3;n++){
                bool my_res = ictrl_left_arm->setControlMode(n+8,VOCAB_CM_FORCE_IDLE);
	            //iMode2->setControlMode(n+8,VOCAB_CM_POSITION);
                if (!my_res){
                    reply.shutdown_reply = reply.FAILURE;
                    break;
                }
	        }
            int nj_right=0;
            pos_right_arm->getAxes(&nj_right);
            // Setting Control Mode - Position
            for(int i=0;i< 8;i++){
                bool my_res = ictrl_right_arm->setControlMode(i,VOCAB_CM_IDLE);
                if (!my_res){
                    reply.shutdown_reply = reply.FAILURE;
                    break;
                }
            }
	        for (size_t n=0;n<3;n++){
                bool my_res = ictrl_right_arm->setControlMode(n+8,VOCAB_CM_FORCE_IDLE);
	            //iMode2->setControlMode(n+8,VOCAB_CM_POSITION);
                if (!my_res){
                    reply.shutdown_reply = reply.FAILURE;
                    break;
                }
	        }
        }
        else if(msg.shutdown_request==msg.TURNON){
            int nj_left=0;
            pos_left_arm->getAxes(&nj_left);
            // Setting Control Mode - Position
            for(int i=0;i< 11;i++){
                bool my_res = ictrl_left_arm->setControlMode(i,VOCAB_CM_POSITION);
                if (!my_res){
                    reply.shutdown_reply = reply.FAILURE;
                    break;
                }
            }
            yarp::sig::VectorOf<int> jntHand;
            jntHand.push_back(8);
            jntHand.push_back(9);
            jntHand.push_back(10);
            //iMode2->setControlModes(3, jntHand.getFirst(), modes.getFirst());
	        for (size_t n=0;n<3;n++){
                ictrl_left_arm->setControlMode(n+8,VOCAB_CM_FORCE_IDLE);
	            bool my_res = ictrl_left_arm->setControlMode(n+8,VOCAB_CM_POSITION);
                if (!my_res){
                    reply.shutdown_reply = reply.FAILURE;
                    break;
                }
	        }
            int nj_right=0;
            pos_right_arm->getAxes(&nj_right);
            // Setting Control Mode - Position
            for(int i=0;i< 8;i++){
                bool my_res = ictrl_right_arm->setControlMode(i,VOCAB_CM_POSITION);
                if (!my_res){
                    reply.shutdown_reply = reply.FAILURE;
                    break;
                }
            }
	        for (size_t n=0;n<3;n++){
                ictrl_right_arm->setControlMode(n+8,VOCAB_CM_FORCE_IDLE);
	            bool my_res = ictrl_right_arm->setControlMode(n+8,VOCAB_CM_POSITION);
                if (!my_res){
                    reply.shutdown_reply = reply.FAILURE;
                    break;
                }
	        }
        }
        server.reply(reply);
    }

    return 0;
}
