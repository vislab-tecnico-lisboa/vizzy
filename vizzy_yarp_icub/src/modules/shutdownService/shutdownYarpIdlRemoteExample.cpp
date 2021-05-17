#include <iostream>
#include <yarp/os/all.h>
#include <yarp/os/Run.h>
#include <vizzy_msgs/MotorsShutdown.h>
#include <vizzy_msgs/MotorsShutdownReply.h>

using namespace yarp::os;

int main(int argc, char * argv[])
{
    Network yarp;
    RpcServer server;
    vizzy_msgs::MotorsShutdown example;
    yarp::os::Run tst;
    server.promiseType(example.getType());
    if (!server.open("/shutdown_yarp_idl@/yarp_idl_shutdown_server")) {
        fprintf(stderr,"Failed to open port\n");
        return 1;
    }

    while (true) {
        vizzy_msgs::MotorsShutdown msg;
        vizzy_msgs::MotorsShutdownReply reply;
        if (!server.read(msg,true)) continue;
        yarp::os::ConstString my_server("/vizzy-desktop");
        yarp::os::ConstString my_tag(":vizzy-desktopyarpidl_rosmsg--name::typ@:yarpidl0");
        bool yarp_idl_running =tst.isRunning(my_server,my_tag);
        if (yarp_idl_running)
            std::cout << "yarpidl_rosmsg is running" << std::endl;
        else
            std::cout << "yarpidl_rosmsg is not running" << std::endl;
        if (msg.shutdown_request==msg.SHUTDOWN){
            int res = tst.sigterm("/vizzy-desktop",":vizzy-desktopyarpidl_rosmsg--name::typ@:yarpidl0");
            if (res==0){
                reply.shutdown_reply=reply.SUCCESS;
                std::cout << "Killed successfully" << std::endl;
            }
            else if (res == -1){
                reply.shutdown_reply=reply.FAILURE;
                std::cout << "Not able to kill it" << std::endl;
            }
        }
        else if(msg.shutdown_request==msg.TURNON){
            yarp::os::Property my_example;
            my_example.put("name",yarp::os::Value("yarpidl_rosmsg --name /typ@/yarpidl"));
            //my_example.put("parameters",yarp::os::Value("--name /typ@/yarpidl"));
            
            int res2 = tst.start(my_server,my_example,my_tag);
            if (res2==0){
                //wait and check for the motors and put the arms down
                reply.shutdown_reply=reply.SUCCESS;
                std::cout << "Started successfully" << std::endl;
            }
            else if (res2 == -1){
                reply.shutdown_reply=reply.FAILURE;
                std::cout << "Not able to start it" << std::endl;
            }
        }
        server.reply(reply);
    }

    return 0;
}
