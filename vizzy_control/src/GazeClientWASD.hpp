/* Joao Avelino. Based on GazeClient.cpp by Rui Figueiredo
*/
#ifndef GAZECLIENTWASD_H
#define GAZECLIENTWASD_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <vizzy_msgs/GazeAction.h>

class GazeClientWASD
{
public:
    GazeClientWASD();
};

#endif // GAZECLIENTWASD_H
