#include <iostream>
#include <yarp/os/all.h>
#include <yarp/os/Run.h>
#include <vizzy_msgs/ShutdownStartYarpRunProcess.h>
#include <vizzy_msgs/ShutdownStartYarpRunProcessReply.h>

using namespace yarp::os;

int main(int argc, char * argv[])
{
    Network yarp;
    RpcServer server;
    vizzy_msgs::ShutdownStartYarpRunProcess example;
    yarp::os::Run tst;
    server.promiseType(example.getType());
    if (!server.open("/shutdownStartProcess@/start_shutdown_server")) {
        fprintf(stderr,"Failed to open port\n");
        return 1;
    }

    while (true) {
        vizzy_msgs::ShutdownStartYarpRunProcess msg;
        vizzy_msgs::ShutdownStartYarpRunProcessReply reply;
        if (!server.read(msg,true)) continue;
        ConstString my_server(msg.server_port);
        ConstString my_tag(msg.tag_str);
        ConstString my_command(msg.command_str);
        bool process_running =tst.isRunning(my_server,my_tag);
        if (process_running)
            std::cout << "Process " << my_command << " is running" << std::endl;
        else
            std::cout << "Process " << my_command << " is not running" << std::endl;
        if (msg.shutdown_request==msg.SHUTDOWN){
            if (process_running){
                int res = tst.sigterm(my_server,my_tag);
                if (res==0){
                    reply.shutdown_reply=reply.SUCCESS;
                    std::cout << "Process " << my_command << " Killed successfully" << std::endl;
                    reply.shutdown_reply = reply.SUCCESS;
                }
                else if (res == -1){
                    reply.shutdown_reply=reply.FAILURE;
                    std::cout << "Not able to kill process " << my_command << std::endl;
                    reply.shutdown_reply = reply.FAILURE;
                }
            }
            else{
                reply.shutdown_reply = reply.SUCCESS;
            }
        }
        else if(msg.shutdown_request==msg.TURNON){
            yarp::os::Property my_example;
            my_example.put("name",yarp::os::Value(my_command));
            //my_example.put("parameters",yarp::os::Value("--name /typ@/yarpidl"));
            if (!process_running){
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
            else{
                reply.shutdown_reply=reply.SUCCESS;
            }
        }
        server.reply(reply);
    }

    return 0;
}
