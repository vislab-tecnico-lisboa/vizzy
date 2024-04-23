// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 *         Joao Avelino <javelino@isr.tecnico.ulisboa.pt>
 *         Plinio Moreno <plinio@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "vizzyArmRoutines.h"

float max_coeffs[3] = { 1.0, 0.53};
float min_coeffs[4] = {0.0, 0.17, 0.25};
float sleeps[7] = {0.65, 0.42, 0.30, 0.22, 0};


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
    hand_force_control = rf.check("fingerControlEnabled", Value("false"),"finger Control Enabled").asBool();
    std::cout << "HAND FORCE CONTROL ENABLED: " << hand_force_control << std::endl;
    setName(moduleName.c_str());

    /* port names */
    commandPortName  = "/" + moduleName + "/" + armName + "/command";

    rosNode = new Node("/" + moduleName + "/" + armName + "/node"); 
  
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

    // Gaze stuff
    bool outputOk_3 = xd_outputPort.topic("/gaze/goal");
    xd_outputPort.setWriteOnly();

     if(!outputOk_3){
        printf("outputOk_3 failed to open\n");
        return -1;
    }
   

    gaze_point_home.goal.fixation_point_error_tolerance = 0.01;

    /*--------------------------------------------------------- 
        Define the points reference frame.
        base_footprint is approximately centered below the front laser,
        in the floor. You can check the frames in Rviz.
                x axis - forward
                y axis - left
                z axis - up  
    -----------------------------------------------------------*/
    gaze_point_home.goal.fixation_point.header.frame_id="base_footprint";

    gaze_point_home.goal.fixation_point.point.x = 1.0;
    gaze_point_home.goal.fixation_point.point.y = 0.0;
    gaze_point_home.goal.fixation_point.point.z = 0.0;

    gaze_point = gaze_point_home;


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

    cout << "The number of axes" << nj << &nj << endl;

	velocities_waving.resize(nj);
    velocities_stretching.resize(nj);
    velocities_handshaking.resize(nj);
    velocities_happy.resize(nj);
    velocities_sad.resize(nj);
    velocities_angry.resize(nj);
    velocities_fear.resize(nj);
    velocities_surprise.resize(nj);
    
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
    velocities_stretching[8] = 40;
    velocities_stretching[9] = 40;
    velocities_stretching[10] = 40;



    // Setting Motor velocities for waving motion

    velocities_handshaking = velocities_waving;
    velocities_handshaking[4] = velocities_waving[4]*1.24;
    velocities_handshaking[6] = velocities_waving[6]*1.24;
    velocities_handshaking[8] = 40;
    velocities_handshaking[9] = 40;
    velocities_handshaking[10] = 40;
    
    // Setting Motor velocities for happy pose  

    velocities_happy[0] = 6;
    velocities_happy[1] = 30;
    velocities_happy[2] = 20;
    velocities_happy[3] = 20;
    velocities_happy[4] = 25;
    velocities_happy[5] = 20;
    velocities_happy[6] = 20;
    velocities_happy[7] = 20;
    velocities_happy[8] = 20;
    velocities_happy[9] = 20;
    velocities_happy[10] = 20;

    // Setting Motor velocities for sad pose
    
    velocities_sad[0] = 15;
    velocities_sad[1] = 15;
    velocities_sad[2] = 15;
    velocities_sad[3] = 15;
    velocities_sad[4] = 15;
    velocities_sad[5] = 15;
    velocities_sad[6] = 15;
    velocities_sad[7] = 15;
    velocities_sad[8] = 15;
    velocities_sad[9] = 15;
    velocities_sad[10] = 15;
    
    // Setting Motor velocities for angry pose
    
    velocities_angry[0] = 20;
    velocities_angry[1] = 30;
    velocities_angry[2] = 20;
    velocities_angry[3] = 20;
    velocities_angry[4] = 30;
    velocities_angry[5] = 20;
    velocities_angry[6] = 20;
    velocities_angry[7] = 20;
    velocities_angry[8] = 20;
    velocities_angry[9] = 20;
    velocities_angry[10] = 20;
    
    // Setting Motor velocities for angry pose
    
    velocities_fear[0] = 20;
    velocities_fear[1] = 30;
    velocities_fear[2] = 20;
    velocities_fear[3] = 20;
    velocities_fear[4] = 30;
    velocities_fear[5] = 20;
    velocities_fear[6] = 20;
    velocities_fear[7] = 20;
    velocities_fear[8] = 20;
    velocities_fear[9] = 20;
    velocities_fear[10] = 20;
    
    // Setting Motor velocities for angry pose
    
    velocities_surprise[0] = 20; 
    velocities_surprise[1] = 30;
    velocities_surprise[2] = 20;
    velocities_surprise[3] = 20;
    velocities_surprise[4] = 30;
    velocities_surprise[5] = 20;
    velocities_surprise[6] = 20;
    velocities_surprise[7] = 20;
    velocities_surprise[8] = 20;
    velocities_surprise[9] = 20;
    velocities_surprise[10] = 20;


    cout << "Module Started! YEEE" << endl;
    // Definition of Poses
    command.resize(nj);
    encoders.resize(nj);
    home_pose.resize(nj);
    wave_home_pose.resize(nj);
    arm_forward_pose.resize(nj);
    grabing_hand_pose.resize(nj);
    pid_hand_pose.resize(nj);
    release_hand_pose.resize(nj);
    arm_down_pose.resize(nj);
    happy_pose.resize(nj);
    sad_pose.resize(nj);
    angry_pose.resize(nj);
    fear_pose_left.resize(nj);
    fear_pose_right.resize(nj);
    surprise_pose.resize(nj);
    surprise_open_pose.resize(nj);
    singing_pose_right.resize(nj);
    singing_pose_left.resize(nj);
    brushing_pose.resize(nj);
    rock_paper_pose.resize(nj);

    while(!encs->getEncoders(encoders.data()))
    {
            Time::delay(0.01);
            cout << "." << endl;
    }
    
    home_pose = encoders;
    // Case 1
    wave_home_pose = home_pose; // To be sure the fingers don't change
    // wave_home_pose[0] =-0.0460945*CTRL_RAD2DEG;
    // wave_home_pose[1] = 1.38844*CTRL_RAD2DEG;
    // wave_home_pose[2] = 1.14261*CTRL_RAD2DEG;
    // wave_home_pose[3] = 0.0*CTRL_RAD2DEG;
    // wave_home_pose[4] = 0.87361*CTRL_RAD2DEG; // wave
    // wave_home_pose[5] = 0.033366*CTRL_RAD2DEG;
    // wave_home_pose[6] = -0.542265*CTRL_RAD2DEG;
    // wave_home_pose[7] = -0.260872*CTRL_RAD2DEG;
    //changed by Ricardo
    wave_home_pose[0] =10;
    wave_home_pose[1] = 0;
    wave_home_pose[2] = 35;
    wave_home_pose[3] = 84.4;
    wave_home_pose[4] = 94.2; // wave
    wave_home_pose[5] = 0;
    wave_home_pose[6] = 0;
    wave_home_pose[7] = 0;
    wave_home_pose[8] = 0;
    wave_home_pose[9] = 0; // wave

    // Case 2 - Arm stretched
    arm_forward_pose[0] =-3.6;
    arm_forward_pose[1] = 73.5;
    arm_forward_pose[2] = 5;
    arm_forward_pose[3] = -8.5;
    arm_forward_pose[4] = 47.25-5.0;
    arm_forward_pose[5] = -10.2;
    arm_forward_pose[6] = 28-3.0;
    arm_forward_pose[7] = 18.2;
    arm_forward_pose[8] = 47.5;
    arm_forward_pose[9] = 45;
    arm_forward_pose[10] = 69.75;


    //Case 3 - Grab hand - fixed position and shaking

    grabing_hand_pose[0] =-3.6;
    grabing_hand_pose[1] = 79;
    grabing_hand_pose[2] = 0;
    grabing_hand_pose[3] = -8.5;
    grabing_hand_pose[4] = 40;
    grabing_hand_pose[5] = -10.2;
    grabing_hand_pose[6] = 28-3.0;
    grabing_hand_pose[7] = 18.2;    
    grabing_hand_pose[8] = 120;
    grabing_hand_pose[9] = 95;
    grabing_hand_pose[10] = 165;

    //Case 4 - Release hand

    release_hand_pose[0] =-3.6;
    release_hand_pose[1] = 79;
    release_hand_pose[2] = 0;
    release_hand_pose[3] = -8.5;
    release_hand_pose[4] = 40;
    release_hand_pose[5] = -10.2;
    release_hand_pose[6] = 28-3.0;
    release_hand_pose[7] = 18.2;    
    release_hand_pose[8] = 47.5;
    release_hand_pose[9] = 45;
    release_hand_pose[10] = 69.75;

    //Case 5 - Grab hand - pid control and shaking
    
    pid_hand_pose[0] =-3.6;
    pid_hand_pose[1] = 79;
    pid_hand_pose[2] = 0;
    pid_hand_pose[3] = -8.5;
    pid_hand_pose[4] = 40;
    pid_hand_pose[5] = -10.2;
    pid_hand_pose[6] = 28-3.0;
    pid_hand_pose[7] = 18.2;    
    pid_hand_pose[8] = 37.5;
    pid_hand_pose[9] = 45;
    pid_hand_pose[10] = 69.75;

    //Case 6 - Arm down pose

    arm_down_pose[0] = 0;
    arm_down_pose[1] = 0;
    arm_down_pose[2] = 5;
    arm_down_pose[3] = 0;
    arm_down_pose[4] = 0;
    arm_down_pose[5] = 0;
    arm_down_pose[6] = 0;
    arm_down_pose[7] = 0;
    arm_down_pose[8] = 0;
    arm_down_pose[9] = 0;
    arm_down_pose[10] = 0;
    
    //Case 7 - Happy pose

    happy_pose[0] = 3.2;
    happy_pose[1] = 129.4;
    happy_pose[2] = 5;
    happy_pose[3] = 1.7;
    happy_pose[4] = 51.5;
    happy_pose[5] = 73.1;
    happy_pose[6] = 0;
    happy_pose[7] = 0;
    happy_pose[8] = 0;
    happy_pose[9] = 0;
    happy_pose[10] = 0;

    //Case 8 - Sad pose

    sad_pose[0] = 7.7;
    sad_pose[1] = 21.7;
    sad_pose[2] = 5;
    sad_pose[3] = 0;
    sad_pose[4] = 0;
    sad_pose[5] = 0;
    sad_pose[6] = 0;
    sad_pose[7] = 0;
    sad_pose[8] = 0;
    sad_pose[9] = 0;
    sad_pose[10] = 0;
    
    //Case 9 - Angry pose
    
    angry_pose[0] = -7.4;
    angry_pose[1] = -72.8;
    angry_pose[2] = 39.2;
    angry_pose[3] = -6.8;
    angry_pose[4] = 94.5;
    angry_pose[5] = 0;
    angry_pose[6] = 0;
    angry_pose[7] = -27.3;
    angry_pose[8] = 0;
    angry_pose[9] = 0;
    angry_pose[10] = 0;
    
    //Case 10 - Fear pose (right arm)
    
    fear_pose_right[0] = 17.8;
    fear_pose_right[1] = 95.5;
    fear_pose_right[2] = 20.3;
    fear_pose_right[3] = -51;
    fear_pose_right[4] = 55.7;
    fear_pose_right[5] = 35.7;
    fear_pose_right[6] = 10.5;
    fear_pose_right[7] = -25.2;
    fear_pose_right[8] = 0;
    fear_pose_right[9] = 0;
    fear_pose_right[10] = 0;
    
    //Case 11 - Fear pose (left arm)
    
    fear_pose_left[0] = -17.7;
    fear_pose_left[1] = 84.9;
    fear_pose_left[2] = 7;
    fear_pose_left[3] = -47.6;
    fear_pose_left[4] = 77.7;
    fear_pose_left[5] = 59.5;
    fear_pose_left[6] = 1.4;
    fear_pose_left[7] = -35;
    fear_pose_left[8] = 0;
    fear_pose_left[9] = 0;
    fear_pose_left[10] = 0;
    
    //Case 12 - Surprise pose
    
    surprise_pose[0] = 17.9;
    surprise_pose[1] = 133.5;
    surprise_pose[2] = 42;
    surprise_pose[3] = -44.2;
    surprise_pose[4] = 73.5;
    surprise_pose[5] = -79.9;
    surprise_pose[6] = -3.5;
    surprise_pose[7] = 25.2;
    surprise_pose[8] = 0;
    surprise_pose[9] = 0;
    surprise_pose[10] = 0;
    
    //Case 12 - Surprise pose - open
    
    surprise_open_pose[0] = 17.9;
    surprise_open_pose[1] = 133.5;
    surprise_open_pose[2] = 42;
    surprise_open_pose[3] = -44.2;
    surprise_open_pose[4] = 59.8;
    surprise_open_pose[5] = -79.9;
    surprise_open_pose[6] = -3.5;
    surprise_open_pose[7] = 16.1;
    surprise_open_pose[8] = 0;
    surprise_open_pose[9] = 0;
    surprise_open_pose[10] = 0;

    //Case 13 - Arm Movement singing (right)

    singing_pose_right= arm_down_pose;
    singing_pose_right[0] = 8.9;
    singing_pose_right[1] = 10.4; //shoulder flexion
    singing_pose_right[2] = 41.2; //shoulder abduction
    singing_pose_right[3] = 20.7 ; //shoulder rotation
    singing_pose_right[4] = 109.1;  //elbow flexion
    singing_pose_right[5] = -80.3;  //forearm pronation


    //Case 14 - Arm Movement singing (left)
    singing_pose_left= arm_down_pose;
    singing_pose_left[0] = 0; //shoulder scapula
    singing_pose_left[1] = 30; //shoulder flexion
    singing_pose_left[2] = 0; //shoulder abduction
    singing_pose_left[3] = -40 ; //shoulder rotation
    singing_pose_left[4] = 80;  //elbow flexion

     //Case 15 - Arm Movement brushing (right)
    brushing_pose = arm_down_pose; //arm_down_pose only sets the shoulder abduction 5 degrees
    brushing_pose[0] = 10; //shoulder scapula
    brushing_pose[1] = 30; //shoulder flexion
    brushing_pose[2] = 45.3; //shoulder abduction
    brushing_pose[3] = 20.3; //shoulder rotation
    brushing_pose[4] = 100; //elbow flexion
    brushing_pose[5] = -80.3;  //forearm pronation
    brushing_pose[8] = 4;  //thumb finger
    brushing_pose[9] = 4;  //index finger
    brushing_pose[10] = 4;  //other two fingers

    //Case 17 - Arm Movement brushing (right)
    dancing_pose = arm_down_pose; //arm_down_pose only sets the shoulder abduction 5 degrees
    dancing_pose[0] = 0; //shoulder scapula
    dancing_pose[1] = 0; //shoulder flexion
    dancing_pose[2] = 74.3; //shoulder abduction
    dancing_pose[3] = -84.3; //shoulder rotation
    dancing_pose[4] = 75.4; //elbow flexion
    dancing_pose[5] = 80;  //forearm pronation

    rock_paper_pose = arm_down_pose;

    int numTries = 0;
    //hand_force_control=false;
    if (hand_force_control){
        while (!force_sensor_port.topic("/tactileForceField")) {
                cerr<< "Failed to connect to subscriber to /tactileForceField\n";
                Time::delay(0.5);
        }
        fingerLimbControl = new ControlThread(&force_sensor_port, encs, pos);
        fingerLimbControl->start();
    }

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
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                Time::delay(2);
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
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                Time::delay(1);
                command = wave_home_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                Time::delay(2);
                for(int i=0;i< 9; i++) {
                    if(i%2==0) {
                        command[4]=wave_home_pose[4]+10;
                        pos->positionMove(command.data());
                        cout << "left" << command[4] << endl;
                    }
                    else {
                        command[4]= wave_home_pose[4]-10;
                        pos->positionMove(command.data());
                        cout << "right" << command[4] << endl;
                    }
                    cout << "value:" << command[4] << endl;
                    Time::delay(1); // Alterado
                }
                // Time::delay(10); // Alterado
                // command=home_pose;
                // pos->positionMove(command.data());
                // while(!done) {
                //     pos->checkMotionDone(&done);
                //     Time::delay(0.00001);   // Alterado
                // }
                cout << "Waving motion DONE" << endl;
                break;
            case 2:
                //Stretch arm forward
                cout << "Stretching arm forward" << endl;
                pos->setRefSpeeds(velocities_stretching.data());
                command = arm_forward_pose;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
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
                pos->setRefSpeeds(velocities_handshaking.data());
                command = grabing_hand_pose;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                while(!done) { 
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;

                cout << "Performing shaking motion" << endl;
                for(int i=0;i< 5; i++) {
                    if(i%2==0) {
                        command[4]= grabing_hand_pose[4]+30*min_coeffs[i/2];
                        command[6]= grabing_hand_pose[6] + 18*min_coeffs[i/2]; //34-28
                        pos->positionMove(command.data());
                    }
                    else {
                        command[4]= grabing_hand_pose[4]+30*max_coeffs[i/2];
                        command[6]= grabing_hand_pose[6]+18*max_coeffs[i/2]; //34-28
                        pos->positionMove(command.data());
                    }
                    Time::delay(sleeps[i]*1.2);
                }

                cout << "Letting go of user hand" << endl;
                pos->setRefSpeeds(velocities_waving.data());
                command = release_hand_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;                
                cout << "Returning to home position" << endl;
                pos->setRefSpeeds(velocities_stretching.data());
                command=home_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                cout << "Handshake performed" << endl;
                break;
            case 4:
                cout << "Asking for shake" << endl;
                pos->setRefSpeeds(velocities_handshaking.data());

                //Just to be sure that it's on the right position...
                command = arm_forward_pose;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;


                for(int i=0;i< 5; i++) {
                    if(i%2==0) {
                        command[4]=arm_forward_pose[4]+30*min_coeffs[i/2];
                        command[6]= arm_forward_pose[6] + 18*min_coeffs[i/2]; //34-28
                        pos->positionMove(command.data());
                    }
                    else {
                        command[4]=arm_forward_pose[4]+30*max_coeffs[i/2];
                        command[6]= arm_forward_pose[6]+18*max_coeffs[i/2]; //34-28
                        pos->positionMove(command.data());
                    }
                    Time::delay(sleeps[i]*1.2);
                }

                break;

            case 5:
                if (hand_force_control)
                {
                    cout << "Handshaking with PID and shaking hand" << endl;
                    xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                    Time::delay(1);
                      //Perform handshake
                    pos->setRefSpeeds(velocities_handshaking.data());
                    command = grabing_hand_pose;
                    pos->positionMove(command.data());
                    cout << "Grabbing user hand" << endl;
                    fingerLimbControl->EnableControl();

                    Time::delay(3.5);

                    cout << "Performing shaking motion" << endl;
                    for(int i=0;i< 5; i++) {
                        if(i%2==0) {
                            //command[4]= grabing_hand_pose[4]+30*min_coeffs[i/2];
                            //command[6]= grabing_hand_pose[6] + 18*min_coeffs[i/2]; //34-28
                            //pos->positionMove(command.data());
                            pos->positionMove(4,grabing_hand_pose[4]+30*min_coeffs[i/2]);
                            pos->positionMove(6,grabing_hand_pose[6]+30*min_coeffs[i/2]);
                        }
                        else {
                            pos->positionMove(4,grabing_hand_pose[4]+30*max_coeffs[i/2]);
                            pos->positionMove(6,grabing_hand_pose[6]+30*max_coeffs[i/2]);
                            //command[4]= grabing_hand_pose[4]+30*max_coeffs[i/2];
                            //command[6]= grabing_hand_pose[6]+18*max_coeffs[i/2]; //34-28
                            //pos->positionMove(command.data());
                        }
                        Time::delay(sleeps[i]*1.2);
                    }

                    fingerLimbControl->DisableControl();

                    cout << "Letting go of user hand" << endl;
                    pos->setRefSpeeds(velocities_waving.data());
                    command = release_hand_pose;
                    pos->positionMove(command.data());
                    while(!done) {
                        pos->checkMotionDone(&done);
                        Time::delay(0.00001);   // Alterado
                    }
                    done = false;                
                    cout << "Returning to home position" << endl;
                    pos->setRefSpeeds(velocities_stretching.data());
                    command=arm_down_pose; //changed from home_pose to arm_down_pose
                    pos->positionMove(command.data());
                    while(!done) {
                        pos->checkMotionDone(&done);
                        Time::delay(0.00001);   // Alterado
                    }
                    cout << "Handshake performed" << endl;
                }
                break;

            case 6:
                //Arm down pose
                cout << "Getting arm down" << endl;
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                Time::delay(1);
                command = velocities_happy;
                command[1] = 20;
                pos->setRefSpeeds(command.data());
                command = arm_down_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Arm down pose" << endl;
                break;

            case 7:
                //Happy pose - less motion
                cout << "Getting happy" << endl;
                
                pos->setRefSpeeds(velocities_happy.data());
                command = happy_pose;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;

                cout << "Happy pose" << endl;
                break;

            case 8:
                //Happy pose - more motion
                cout << "Getting happy" << endl;
                
                pos->setRefSpeeds(velocities_happy.data());
                command = happy_pose;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;

                for(int i=0;i<6; i++) {
                    if(i%2==0) {
                        command[1]=happy_pose[1]-15;
                        command[4]=happy_pose[4]+10;
                        pos->positionMove(command.data());
                    }
                    else {
                        command[1]=happy_pose[1]+15;
                        command[4]=happy_pose[4]-10;
                        pos->positionMove(command.data());
                    }
                    Time::delay(0.7);
                }

                cout << "Happy pose" << endl;
                break;

            case 9:
                //Sad pose
                cout << "Getting sad" << endl;
                gaze_point.goal.fixation_point.point.y = -1.0; //left
                gaze_point.goal.fixation_point.point.z = 0.0; //up
                pos->setRefSpeeds(velocities_sad.data());
                command = sad_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;

                //Gaze controller
                xd_outputPort.write(gaze_point);
                
                gaze_point.goal.fixation_point.point.y = 1.0; //left
                Time::delay(2);                    
                xd_outputPort.write(gaze_point);               


                Time::delay(2);
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                cout << "Sad pose" << endl;
                break;

            case 10:
                //Angry pose
                cout << "Getting angry" << endl;
                
                pos->setRefSpeeds(velocities_angry.data());
                command = angry_pose;
                xd_outputPort.write(gaze_point_home);
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Angry pose" << endl;
                break;

            case 11:
                //Fear pose (right arm)
                cout << "Getting fear (right arm)" << endl;
                
                pos->setRefSpeeds(velocities_fear.data());
                command = fear_pose_right;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Fear pose (right arm)" << endl;
                break;

            case 12:
                //Fear pose (left arm)
                cout << "Getting fear (left arm)" << endl;
                
                pos->setRefSpeeds(velocities_fear.data());
                command = fear_pose_left;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Fear pose (left arm)" << endl;
                break;

            case 13:
                //Surprise pose
                cout << "Getting surprised" << endl;
                
                pos->setRefSpeeds(velocities_surprise.data());
                command = surprise_pose;
                pos->positionMove(command.data());
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Surprise pose" << endl;
                break;

            case 14:
                //Stretch arm forward with hand open
                cout << "Stretching arm forward" << endl;
                
                pos->setRefSpeeds(velocities_stretching.data());
                command = arm_forward_pose;
                command[5] = -40;
                command[8] = 0;
                command[9] = 0;
                command[10] = 0;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Arm stretched forward with hand open" << endl;
                break;

            case 15:
                //Surprise pose - open
                cout << "Getting surprised - open pose" << endl;
                
                pos->setRefSpeeds(velocities_surprise.data());
                command = surprise_open_pose;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0
                while(!done) {
                    pos->checkMotionDone(&done);
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                cout << "Surprise open pose" << endl;
                break;

            case 16:
                cout << "Starting to sing right" << endl;
                pos->setRefSpeeds(velocities_surprise.data());
                command = singing_pose_right;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0    
                while(!done) {
                    pos->checkMotionDone(&done);
                    
                    Time::delay(0.00001);   // Alterado
                }
                done = false;

                cout << "Singing pose" << endl;
                break;
                
            case 17:
                cout << "Starting to sing left" << endl;
                pos->setRefSpeeds((velocities_surprise.data()));
                command = singing_pose_left;
                pos->positionMove(command.data());
                // To move the head

                gaze_point.goal.fixation_point.point.x = 0.0; //forward
                gaze_point.goal.fixation_point.point.y = 0.0; //left
                gaze_point.goal.fixation_point.point.z = 0.7; //up
                
                Time::delay(2);
                while(!done) {
                    pos->checkMotionDone(&done);
                    
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                xd_outputPort.write(gaze_point);

                gaze_point.goal.fixation_point.point.x = 0.0; //forward
                gaze_point.goal.fixation_point.point.y = -0.08; //left look to the right
                gaze_point.goal.fixation_point.point.z = 0.0; //up
                Time::delay(1);

                xd_outputPort.write(gaze_point); //
                cout << "Singing pose" << endl;
                break;

            case 18:
                // Brushing teeth (right arm)
                cout << "Starting to brush teeth right" << endl;

                pos->setRefSpeeds(velocities_surprise.data());
                command = brushing_pose;
                pos->positionMove(command.data());
                xd_outputPort.write(gaze_point_home); //Gaze to 10 0 0

                Time::delay(2);
                while(!done) {
                    pos->checkMotionDone(&done);
                    
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                for(int i=0;i< 9; i++) {
                    if(i%2==0) {
                        //command[4]=arm_forward_pose[4]+30*min_coeffs[i/2];
                        command[3]= brushing_pose[3] + 8; //34-28
                        pos->positionMove(command.data());
                        Time::delay(0.5);
                        while(!done) {
                            pos->checkMotionDone(&done);
                            
                            Time::delay(0.0001);   // Alterado
                        }
                        done = false;
                    }
                    else {
                        //command[4]=arm_forward_pose[4]+30*max_coeffs[i/2];
                        command[3]= brushing_pose[3] - 8; //34-28
                        pos->positionMove(command.data());
                        Time::delay(0.5);
                        while(!done) {
                            pos->checkMotionDone(&done);
                            
                            Time::delay(0.0001);   // Alterado
                        }
                        done = false;
                    }
                }


                cout << "Brushing pose" << endl;
                break;


            case 19:
                // Dancing right
                cout << "Starting to dance" << endl;
                pos->setRefSpeeds(velocities_surprise.data());
                command = dancing_pose;
                pos->positionMove(command.data());
                Time::delay(2);
                while(!done) {
                    pos->checkMotionDone(&done);
                    
                    Time::delay(0.00001);   // Alterado
                }
                                
                gaze_point.goal.fixation_point.point.z = -0.05; //up
                xd_outputPort.write(gaze_point); //Gaze movement

                Time::delay(1);
                done = false;

                for(int i=0;i < 4; i++) {
                    
                    command[3] += 30; //34-28
                    cout << "Shoulder rotation " << command[3] << endl;
                    pos->positionMove(command.data());
                    gaze_point.goal.fixation_point.point.z += 0.4; //up
                    cout << "Gaze tilt" << gaze_point.goal.fixation_point.point.z  << endl;
                    Time::delay(2);
                    xd_outputPort.write(gaze_point); //Gaze movement
                    
                    while(!done) {
                        pos->checkMotionDone(&done);
                        
                        Time::delay(0.0001);   // Alterado
                    }
                    Time::delay(1);
                    done = false;
                    
                }

                cout << "Dancing pose" << endl;
                break;

            case 20:
                // Dancing left
                cout << "Starting to dance" << endl;
                pos->setRefSpeeds(velocities_surprise.data());
                command = dancing_pose;
                pos->positionMove(command.data());
                Time::delay(2);
                while(!done) {
                    pos->checkMotionDone(&done);
                    
                    Time::delay(0.00001);   // Alterado
                }
                done = false;
                Time::delay(10);
                for(int i=0;i < 4; i++) {
                    command[3] += 30; //34-28
                    cout << "Shoulder rotation " << command[3] << endl;
                    pos->positionMove(command.data());
                    Time::delay(2);
                    while(!done) {
                        pos->checkMotionDone(&done);
                        
                        Time::delay(0.0001);   // Alterado
                    }
                   
                    Time::delay(1);
                    done = false;      
                }
                cout << "Dancing pose" << endl;
                break;
                
            case 21: //Rock paper scissors pose
                    //To implement
                cout << "Starting game" << endl;
                int random_num;

                pos->setRefSpeeds(velocities_surprise.data());
                command = rock_paper_pose;
                

                random_num = std::rand() % 3;
                if (random_num == 0){
                    cout << "Paper" << endl;
                }
                if (random_num == 1){
                    command[8] = 125;
                    command[9] = 125;
                    command[10] = 125; 
                    cout << "Rock" << endl;
                }
                if (random_num == 2){
                    command[8] = 125;
                    command[9] = 125;                    
                    cout << "Scissors" << endl;
                } 

                pos->positionMove(command.data());
                Time::delay(2);
                while(!done) {
                    pos->checkMotionDone(&done);
                    
                    Time::delay(0.00001);   // Alterado
                }
                done = false;            

                cout << "Game pose" << endl;
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
