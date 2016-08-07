#ifndef GAZECLIENT_H
#define GAZECLIENT_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <vizzy_msgs/GazeAction.h>

class GazeClient
{
public:
    GazeClient();
};

#endif // GAZECLIENT_H
