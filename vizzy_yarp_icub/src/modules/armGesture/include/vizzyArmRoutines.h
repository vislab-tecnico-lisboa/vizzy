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

constexpr std::array<double, 11> letter_start_open_pose_arr    {0.494, -0.733, 15.400, 1.700, 51.449, 0.002, 0.000, 0.001, 0.000,  0.000,    0.000};
constexpr std::array<double, 11> letter_start_closed_pose_arr  {0.494, -0.733, 15.400, 1.700, 51.449, 0.002, 0.000, 0.001, 0.000,  0.000,   209.081};
constexpr std::array<double, 11> letter_finnish_closed_pose_arr{1.539, 99.598, 9.799,  0.000, 16.800, 0.000, 0.000, 21.701,0.000,  0.000,   209.081};
constexpr std::array<double, 11> letter_finnish_open_pose_arr  {1.539, 99.598, 9.799,  0.000, 16.800, 0.000, 0.000, 21.701,0.000,  0.000,   0.000};

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

    double joint_mod{0.0};

    yarp::sig::Vector command, encoders, home_pose, wave_home_pose, grabing_hand_pose, arm_forward_pose, handshaking_pose, release_hand_pose, pid_hand_pose, 
                      letter_start_open_pose{letter_start_open_pose_arr.size(), letter_start_open_pose_arr.data()},
                      letter_start_closed_pose{letter_start_closed_pose_arr.size(), letter_start_closed_pose_arr.data()},
                      letter_finnish_closed_pose{letter_finnish_closed_pose_arr.size(), letter_finnish_closed_pose_arr.data()},
                      letter_finnish_open_pose{letter_finnish_open_pose_arr.size(), letter_finnish_open_pose_arr.data()};

;
    yarp::sig::Vector velocities_waving, velocities_stretching, velocities_handshaking;
    
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
