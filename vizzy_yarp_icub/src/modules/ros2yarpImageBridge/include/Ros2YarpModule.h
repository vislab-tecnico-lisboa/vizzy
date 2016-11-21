/*
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *            Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ROS2YARP_MODULE_H__
#define __ROS2YARP_MODULE_H__

#include <string>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>

#include "sensor_msgs_Image.h"

using namespace yarp::os;
using namespace yarp::sig;

class Ros2YarpModule : public RFModule
{
private:
    std::string moduleName;
    double mydelay;
    std::string inputImagePortName;
    std::string outputImagePortName;
    Node *rosNode;
    Subscriber<sensor_msgs_Image> inputImagePort;
    BufferedPort<ImageOf<PixelRgb> > outputImagePort;
    double t;

public:
    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();
};

#endif // __ROS2YARP_MODULE_H__
