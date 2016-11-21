/*
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *            Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include "Ros2YarpModule.h"

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;

    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("ros2yarp");
    rf.setDefaultConfigFile("ros2yarp.ini");
    rf.configure(argc, argv);

    if(rf.check("help"))
    {
        yInfo("Available options:");
        yInfo("--name prefix (default ros2yarp)");
        yInfo("--delay seconds (default 0.05)");
        return 0; // EXIT_SUCCESS
    }

    if (! yarp::os::Network::checkNetwork())
    {
        yError("yarpserver not available!");
        return 1; // EXIT_FAILURE
    }

    Ros2YarpModule mod;
    return mod.runModule(rf);
}
