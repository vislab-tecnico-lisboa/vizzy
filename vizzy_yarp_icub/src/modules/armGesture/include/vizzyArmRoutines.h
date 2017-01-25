// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef VIZZYARMROUTINES_H
#define VIZZYARMROUTINES_H

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Node.h>
#include <yarp/os/Subscriber.h>
#include <fstream>
#include <string>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include <yarp/math/Math.h>
#include <cmath>

#include "Int16.h"
#include "VIZZYARMROUTINES_IDL.h"

#define CTRL_RAD2DEG 180.0/M_PI;

using namespace yarp::os;
using namespace std;
using namespace yarp::dev;

class VizzyArmRoutines: public RFModule, public VIZZYARMROUTINES_IDL {
    string moduleName;
    string robotName;
    string armName;
    /*Name of ports to be open*/
    string commandPortName, handlerPortName;
    // RPC server
    RpcServer handlerPort;
    // Node
    Node *rosNode;
    // Subscriber
    Subscriber<Int16> command_sub;
    // Stuff to Arm Controller
    Property options;
    PolyDriver robotDevice;
    IPositionControl *pos;
    IEncoders *encs;
    IControlMode2 *ictrl;
    yarp::sig::Vector command, encoders, home_pose, wave_home_pose;
private:
    bool _closing;
public:
    
    double getPeriod();
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();
    bool interruptModule();
    bool close();

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    bool quit();
};
#endif // VIZZYARMROUTINES_H
