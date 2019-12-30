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
#include "VIZZYARMROUTINES_IDL.h"

#include "Int16.h"

#include <iostream>
#include <iomanip>
#include <string>
#include "ControlThread.h"
#include <TactSensor.h>
#include <TactSensorArray.h>
#include <yarp/os/Subscriber.h>

#include "pid.h"
#include <cmath> 

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
    yarp::os::Subscriber<TactSensorArray> force_sensor_port;
    // Stuff to Arm Controller
    Property options;
    PolyDriver robotDevice;
    IPositionControl *pos;
    IEncoders *encs;
    IControlMode2 *ictrl;
    ControlThread *fingerLimbControl;
    yarp::sig::Vector command, encoders, home_pose, wave_home_pose, grabing_hand_pose, arm_forward_pose, handshaking_pose, release_hand_pose, pid_hand_pose, arm_down_pose, happy_pose, sad_pose, angry_pose, fear_pose_left, fear_pose_right, surprise_pose, surprise_open_pose;
    yarp::sig::Vector velocities_waving, velocities_stretching, velocities_handshaking, velocities_happy, velocities_sad, velocities_angry, velocities_fear, velocities_surprise;
    
private:
    bool _closing;
    bool hand_force_control;
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
