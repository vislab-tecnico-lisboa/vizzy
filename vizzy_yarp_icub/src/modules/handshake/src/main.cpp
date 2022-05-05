// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Node.h>
#include <yarp/os/Subscriber.h>
#include <fstream>
#include <string>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include "runControl.h"


#include <TactSensorArray.h>
#include <TactSensor.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

int main(int argc, char *argv[]) {

    std::string moduleName="handshake";
    std::string remotePorts="/";
    std::string robotName = "vizzy";
    std::string armName="right";
    Network yarp;
    Property options;
    PolyDriver robotDevice;
    IPositionControl *pos;
    IEncoders *encs;
    IControlMode2 *ictrl;
    RunControl *fingerLimbControl;
    yarp::os::Subscriber<TactSensorArray> force_sensor_port;
    yarp::sig::Vector grabing_hand_pose, velocities_waving, velocities_handshaking, release_hand_pose,velocities_stretching,home_pose, command, encoders;

    float max_coeffs[3] = { 1.0, 0.53};
    float min_coeffs[4] = {0.0, 0.17, 0.25};
    float sleeps[7] = {0.65, 0.42, 0.30, 0.22, 0};


    if(! yarp.checkNetwork() ) {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return 1;
    }
    yarp::os::Node node("/yarp/motionTest");

    Node *rosNode = new Node("/" + moduleName + "/" + armName + "/node"); 


    // Motor Stuff - Right Arm
    options.put("robot","vizzy");
    remotePorts+=robotName;
	remotePorts+="/";
	remotePorts+=armName;
    remotePorts+="_shoulder_arm";

    std::string localPorts="/my_module/controller/" + armName + "SArm";
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
    cout << "Drivers initialized" << endl;
    bool done;
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }
    int nj=0;
    pos->getAxes(&nj);
    cout << "nj: " << nj << endl;
    velocities_waving.resize(nj);
    velocities_stretching.resize(nj);
    velocities_handshaking.resize(nj);
    command.resize(nj);
    encoders.resize(nj);
    home_pose.resize(nj);
    grabing_hand_pose.resize(nj);
    release_hand_pose.resize(nj);

    while(!encs->getEncoders(encoders.data()))
    {
        Time::delay(0.01);
        cout << "." << endl;
    }


    grabing_hand_pose[0] =-3.6;
    grabing_hand_pose[1] = 79;
    grabing_hand_pose[2] = 0;
    grabing_hand_pose[3] = -8.5;
    grabing_hand_pose[4] = 35;
    grabing_hand_pose[5] = -10.2;
    grabing_hand_pose[6] = 28-3.0;
    grabing_hand_pose[7] = 18.2;    
    grabing_hand_pose[8] = 120;
    grabing_hand_pose[9] = 95;
    grabing_hand_pose[10] = 165;

    release_hand_pose[0] =-3.6;
    release_hand_pose[1] = 79;
    release_hand_pose[2] = 0;
    release_hand_pose[3] = -8.5;
    release_hand_pose[4] = 35;
    release_hand_pose[5] = -10.2;
    release_hand_pose[6] = 28-3.0;
    release_hand_pose[7] = 18.2;    
    release_hand_pose[8] = 47.5;
    release_hand_pose[9] = 45;
    release_hand_pose[10] = 69.75;


    for (int i=0;i< nj;i++)
	    velocities_waving[i] = 21.0;
    velocities_waving[0] = 10.0; // shoulder
    velocities_waving[4] = 30.0; // prosupination - wave joint


    velocities_handshaking = velocities_waving;
    //velocities_handshaking[4] = velocities_waving[4]*1;
    //velocities_handshaking[6] = velocities_waving[6]*1;
    velocities_handshaking[4] = velocities_waving[4]*1.24;
    velocities_handshaking[6] = velocities_waving[6]*1.24;  
    velocities_handshaking[8] = 40;
    velocities_handshaking[9] = 40;
    velocities_handshaking[10] = 40;

    velocities_stretching[0] = 15;
    velocities_stretching[1] = 40;
    velocities_stretching[2] = 15;
    velocities_stretching[3] = 15;
    velocities_stretching[4] = 15;
    velocities_stretching[5] = 20;
    velocities_stretching[6] = 20;
    velocities_stretching[7] = 10;
    velocities_stretching[8] = 40;
    velocities_stretching[9] = 40;
    velocities_stretching[10] = 40;

    home_pose = encoders;

    while(!force_sensor_port.topic("forces")){
        cerr<< "Failed to connect to subscriber to /forces\n";
        Time::delay(0.5);
    }
    fingerLimbControl = new RunControl(&force_sensor_port,encs,pos);
    fingerLimbControl->start();

    cout << "home pose 0: " << home_pose[0] << endl;
    cout << "home pose 1: " << home_pose[1] << endl;
    cout << "home pose 2: " << home_pose[2] << endl;
    cout << "home pose 3: " << home_pose[3] << endl;
    cout << "home pose 4: " << home_pose[4] << endl;
    cout << "home pose 5: " << home_pose[5] << endl;
    cout << "home pose 6: " << home_pose[6] << endl;
    cout << "home pose 7: " << home_pose[7] << endl;
    cout << "home pose 8: " << home_pose[8] << endl;
    cout << "home pose 9: " << home_pose[9] << endl;
    cout << "home pose 10: " << home_pose[10] << endl;


    char input;
    cout << "Waiting for input" << endl;
    cin>>input;
    while (input!='q'){
        if (input=='f'){

            fingerLimbControl->EnableControl();
            
            Time::delay(5);

            fingerLimbControl->DisableControl();


          /*  cout << "Performing handshake..." << endl;
            cout << "Grabbing user hand" << endl;
            pos->setRefSpeeds(velocities_handshaking.data());
            command = grabing_hand_pose;
            
            cout << "grabing hand pose: " << grabing_hand_pose[4] << endl;

            pos->positionMove(command.data());

            while(!done) { 
                pos->checkMotionDone(&done);
                Time::delay(0.00001);   // Alterado
            }
            done = false;

            //this time will be taken out once i figure out the time it takes for the fingers to close 
            Time::delay(2);

            cout << "Performing shaking motion" << endl;
            for(int i=0;i< 5; i++) {
                if(i%2==0) {
                    //command[4]= grabing_hand_pose[4]+30*min_coeffs[i/2];
                    command[4]=35;
                    pos->positionMove(command.data());
                    cout << "command4: " << command[4] << endl;
                    
                }
                else {
                    //command[4]= grabing_hand_pose[4]+30*max_coeffs[i/2];
                    command[4]=60;
                    pos->positionMove(command.data());
                    cout << "command4: " << command[4] << endl;
                }
                //Time::delay(sleeps[i]*1.2);
                Time::delay(0.7);
                //cout << "waiting time:" << sleeps[i]*1.2 << endl;
            }


            cout << "Letting go of user hand" << endl;
            pos->setRefSpeeds(velocities_waving.data());
            command = release_hand_pose;
            cout << "release hand pose: " << command[4] << endl;
            pos->positionMove(command.data());
            while(!done) {
                pos->checkMotionDone(&done);
                Time::delay(0.00001);   // Alterado
            }
            done = false;  

            cout << "Returning to home position" << endl;
            pos->setRefSpeeds(velocities_stretching.data());
            command=home_pose;
            cout << "home pose: " << command[4] << endl;
            pos->positionMove(command.data());


            while(!done) {
                pos->checkMotionDone(&done);
                Time::delay(0.00001);   // Alterado
            }
            cout << "Handshake performed" << endl;*/
        }
        cin>>input;
    }

    return 0;
}