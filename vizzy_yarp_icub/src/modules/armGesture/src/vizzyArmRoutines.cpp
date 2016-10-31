// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "vizzyArmRoutines.h"
#include "Int16.h"

using namespace yarp::os;
using namespace std;
using namespace yarp::math;
using namespace yarp::sig;
// VizzyArmRoutines Module
double VizzyArmRoutines::getPeriod() {
    return 0.0;
}

/*IDL Functions*/
bool VizzyArmRoutines::quit() {
    cout << "Received Quit command in RPC" << endl;
    _closing = true;
    return true;
}

bool VizzyArmRoutines::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}
// End IDL functions

//Configure
bool VizzyArmRoutines::configure(yarp::os::ResourceFinder &rf) {
    _closing = false;
    /* module name */
	Vector tmp;
    moduleName = rf.check("name", Value("vizzyArmRoutines"),
                          "Module name (string)").asString();
    robotName = rf.check("robot", Value("vizzy"),"Robot name (string)").asString();
	armName = rf.check("arm", Value("right"),"Arm name (string)").asString();
    setName(moduleName.c_str());

    /* port names */
    commandPortName  = "/" + moduleName + "/command";

    rosNode = new Node("/" + moduleName + "/node"); 
    //Open Topic
    command_sub.setReadOnly();
    if (!command_sub.topic(
             commandPortName.c_str())) {
        cout << getName() << ": unable to open port"
        << commandPortName << endl;
        return -1;
    }

    handlerPortName = "/" + moduleName + "/rpc:i";
    handlerPort.open(handlerPortName.c_str());
    attach(handlerPort);

    // Motor Stuff - Right Arm
    options.put("robot",robotName.c_str());
    std::string remotePorts="/";
    remotePorts+=robotName;
	remotePorts+="/";
	remotePorts+=armName;
    remotePorts+="_shoulder_arm";

    std::string localPorts="/" + moduleName + "/controller/" + armName + "SArm";
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());           //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to
    
    robotDevice.open(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }
    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);
    ok = ok && robotDevice.view(ictrl);
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }
    int nj=0;
    pos->getAxes(&nj);
	tmp.resize(nj);
    // Setting Control Mode - Position
    for(int i=0;i< nj;i++)
        ictrl->setControlMode(i,VOCAB_CM_POSITION);
	// Setting Motor Velocities
	for (int i=0;i< nj;i++)
	    tmp[i] = 21.0;
	pos->setRefSpeeds(tmp.data());
	// 
	pos->setRefSpeed(0,10.0); // shoulder
	pos->setRefSpeed(4,30.0); // prosupination - wave joint

    cout << "Module Started! YEEE" << endl;
    // Definition of Poses
    command.resize(nj);
    encoders.resize(nj);
    home_pose.resize(nj);
    wave_home_pose.resize(nj);
    while(!encs->getEncoders(encoders.data()))
    {
            Time::delay(0.01);
            cout << "." << endl;
    }
    home_pose = encoders;

    // Case 1
    wave_home_pose = home_pose; // To be sure the fingers don't change
    wave_home_pose[0] =-0.0460945*CTRL_RAD2DEG;
    wave_home_pose[1] = 1.38844*CTRL_RAD2DEG;
    wave_home_pose[2] = 1.14261*CTRL_RAD2DEG;
    wave_home_pose[3] = 0.0*CTRL_RAD2DEG;
    wave_home_pose[4] = 0.87361*CTRL_RAD2DEG; // wave
    wave_home_pose[5] = 0.033366*CTRL_RAD2DEG;
    wave_home_pose[6] = -0.542265*CTRL_RAD2DEG;
    wave_home_pose[7] = -0.260872*CTRL_RAD2DEG;

    // Case 2

	// TO BE IMPLEMENTED
    return true;
}

/**Update**/
bool VizzyArmRoutines::updateModule() {

    Int16 *commandreceived;
//    if(commandPort.getInputCount()<=0)
//        return true;

    commandreceived = command_sub.read(false);
    bool done = false;
    if (commandreceived!=NULL) {
        cout << "I received command" << commandreceived->data << endl;
        switch(commandreceived->data) {
            case 1: 
                cout << "Waving motion" << endl;

                command = wave_home_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                Time::delay(0.5);
                for(int i=0;i< 5; i++) {
                    if(i%2==0) {
                        command[4]=wave_home_pose[4]+10;
                        pos->positionMove(command.data());
                        cout << "left" << endl;
                    }
                    else {
                        command[4]=wave_home_pose[4]-10;
                        pos->positionMove(command.data());
                        cout << "right" << endl;
                    }
                    Time::delay(0.6);
                }
                command=home_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                cout << "Waving motion DONE" << endl;
                break;
            case 2:
                break;
            default:
                cout << "unknown command" << endl;
        }
    }
    

    return !_closing;

}

bool VizzyArmRoutines::interruptModule() {
    cout << "Interrupting your module, for port cleanup" << endl;

    command_sub.interrupt();
    return true;
}

bool VizzyArmRoutines::close() {

    /* optional, close port explicitly */
    cout << "Calling close function\n";

    command_sub.close();
    return true;
}
