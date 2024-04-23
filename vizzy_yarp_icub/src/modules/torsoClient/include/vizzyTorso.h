// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Authors: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 *         Joao Avelino <javelino@isr.tecnico.ulisboa.pt>
 *         Plinio Moreno <plinio@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef VIZZYTORSO_H
#define VIZZYTORSO_H

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
#include "VIZZYTORSO_IDL.h"
#include "Int16.h"
#include "Float64.h"
#include <iostream>
#include <iomanip>
#include <string>

#include <cmath> 


using namespace yarp::os;
using namespace std;
using namespace yarp::dev;

class VizzyTorso: public RFModule, public VIZZYTORSO_IDL {
       
    string moduleName;
    string robotName;
    string armName;
    /*Name of ports to be open*/
    string commandPortName, handlerPortName;
    // RPC server
    RpcServer handlerPort;
    // Node
    Node *rosNode_torso;
    // Subscriber
    Subscriber<Int16> command_sub_torso;
    // Stuff to Torso Controller
    Property options_torso;
    PolyDriver robotDevice_torso;
    IPositionControl *pos_torso=0;
    
    IEncoders *encs_torso;
    IControlMode2 *ictrl_torso;

    yarp::sig::Vector command, encoders_torso, torso_pose;
    yarp::sig::Vector velocities_torso;
    
private:
    bool _closing;
    bool performMotion(yarp::sig::Vector pos);
public:
    
    double getPeriod();
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();
    bool interruptModule();
    bool close();
    void setJointsVelocities(yarp::sig::Vector &velocities);

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    bool quit();
};
#endif // VIZZYARMROUTINES_H
