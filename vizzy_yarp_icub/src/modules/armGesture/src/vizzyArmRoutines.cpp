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
	velocities_waving.resize(nj);
    velocities_stretching.resize(nj);
    velocities_handshaking.resize(nj);
    // Setting Control Mode - Position
    for(int i=0;i< nj;i++)
        ictrl->setControlMode(i,VOCAB_CM_POSITION);
	// Setting Motor Velocities for waving motion
	for (int i=0;i< nj;i++)
	    velocities_waving[i] = 21.0;
    velocities_waving[0] = 10.0; // shoulder
    velocities_waving[4] = 30.0; // prosupination - wave joint

    // Setting Motor velocities for arm stretching motion

    velocities_stretching[0] = 15;
    velocities_stretching[1] = 40;
    velocities_stretching[2] = 15;
    velocities_stretching[3] = 15;
    velocities_stretching[4] = 15;
    velocities_stretching[5] = 20;
    velocities_stretching[6] = 20;
    velocities_stretching[7] = 10;
    velocities_stretching[8] = 20;
    velocities_stretching[9] = 20;
    velocities_stretching[10] = 35;


    // Setting Motor velocities for handshaking motion

    velocities_handshaking = velocities_waving;





    cout << "Module Started! YEEE" << endl;
    // Definition of Poses
    command.resize(nj);
    encoders.resize(nj);
    home_pose.resize(nj);
    wave_home_pose.resize(nj);
    arm_forward_pose.resize(nj);
    grabing_hand_pose.resize(nj);
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

    // Case 2 - Arm stretched
    arm_forward_pose[0] =2.25*CTRL_RAD2DEG;
    arm_forward_pose[1] = 110*CTRL_RAD2DEG;
    arm_forward_pose[2] = 2.06*CTRL_RAD2DEG;
    arm_forward_pose[3] = -61.18*CTRL_RAD2DEG;
    arm_forward_pose[4] = 15.68*CTRL_RAD2DEG;
    arm_forward_pose[5] = -69.63*CTRL_RAD2DEG;
    arm_forward_pose[6] = 34*CTRL_RAD2DEG;
    arm_forward_pose[7] = -0.7*CTRL_RAD2DEG;
    arm_forward_pose[8] = 37.5*CTRL_RAD2DEG;
    arm_forward_pose[9] = 45*CTRL_RAD2DEG;
    arm_forward_pose[10] = 69.75*CTRL_RAD2DEG;

    //Case 3 - Grab hand
    grabing_hand_pose = arm_forward_pose;
    grabing_hand_pose[8] = 111*CTRL_RAD2DEG;
    grabing_hand_pose[9] = 103.5*CTRL_RAD2DEG;
    grabing_hand_pose[10] = 193.5*CTRL_RAD2DEG;



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

            case 0:
                //Home position
                cout << "Going to home position" << endl;
                pos->setRefSpeeds(velocities_stretching.data());
                command = home_pose;

                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Went to home position" << endl;
                break;

            case 1: 
                cout << "Waving motion" << endl;

                pos->setRefSpeeds(velocities_waving.data());

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
                //Stretch arm forward
                cout << "Stretching arm forward" << endl;
                
                pos->setRefSpeeds(velocities_stretching.data());
                command = arm_forward_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Arm stretched forward" << endl;
                break;
            
            case 3:
                //Perform handshake
                cout << "Performing handshake..." << endl;
                cout << "Grabbing user hand" << endl;
                pos->setRefSpeeds(velocities_waving.data());
                command = grabing_hand_pose;
                while(!done) { 
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;

                cout << "Performing waving motion" << endl;
                for(int i=0;i< 5; i++) {
                    if(i%2==0) {
                        command[4]=wave_home_pose[4]+10;
                        pos->positionMove(command.data());
                    }
                    else {
                        command[4]=wave_home_pose[4]-10;
                        pos->positionMove(command.data());
                    }
                    Time::delay(0.6);
                }

                cout << "Letting go of user hand" << endl;



                cout << "Returning to home position" << endl;


                cout << "Handshake performed" << endl;
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