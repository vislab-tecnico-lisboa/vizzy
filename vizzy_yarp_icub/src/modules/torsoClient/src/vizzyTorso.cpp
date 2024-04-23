// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2024 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Ricardo Rodrigues <ricardo.ramos.rodrigues@tecnico.ulisboa.pt>
 *         Plinio Moreno <plinio@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "vizzyTorso.h"


using namespace yarp::os;
using namespace std;
using namespace yarp::math;
using namespace yarp::sig;

// VizzyTorso Module
double VizzyTorso::getPeriod() {
    return 0.0;
}

/*IDL Functions*/
bool VizzyTorso::quit() {
    cout << "Received Quit command in RPC" << endl;
    _closing = true;
    return true;
}

bool VizzyTorso::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}
// End IDL functions

//Configure
bool VizzyTorso::configure(yarp::os::ResourceFinder &rf) {
    _closing = false;
    moduleName = rf.check("name", Value("vizzyTorso"),
                          "Module name (string)").asString();
    robotName = rf.check("robot", Value("vizzy"),"Robot name (string)").asString();

    /* port names */
    commandPortName  = "/" + moduleName + "/command";

    rosNode_torso = new Node("/" + moduleName + "/node"); 
  
    //Open Topic
    command_sub_torso.setReadOnly();
    if (!command_sub_torso.topic(
             commandPortName.c_str())) {
        cout << getName() << ": unable to open Torso port"
        << commandPortName << endl;
        return -1;
    }

    handlerPortName = "/" + moduleName + "/rpc:i";
    handlerPort.open(handlerPortName.c_str());
    attach(handlerPort);
        
    options_torso.put("robot",robotName.c_str());
    std::string remotePortsTorso="/";
    remotePortsTorso+=robotName;
    remotePortsTorso+="/torso";

    options_torso.put("device", "remote_controlboard");
    options_torso.put("local", "/vizzy/torso_pos_interface");           //local port names
    options_torso.put("remote", remotePortsTorso.c_str());

    //options_torso.put("part", "torso");

    robotDevice_torso.open(options_torso);
    if (!robotDevice_torso.isValid()) {
        printf("Device Torso not available.  Here are the known devices:");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    bool ok;

    ok = robotDevice_torso.view(pos_torso);
    cout << "Value of ok" << ok << endl;
    ok = ok && robotDevice_torso.view(encs_torso);
    ok = ok && robotDevice_torso.view(ictrl_torso);



    if (!ok) {
        printf("Problems acquiring torso interface\n");
        return 0;
    }
    int nj_torso=0;
    pos_torso->getAxes(&nj_torso);

    cout << "The number of torso axes" << nj_torso << &nj_torso << endl;

    velocities_torso.resize(nj_torso); //Setting velocities for torso

    // Setting Control Mode - Position
    
    ictrl_torso->setControlMode(0,VOCAB_CM_POSITION);


    cout << "Module of Torso Started! YEEE" << endl;
    encoders_torso.resize(nj_torso);
    torso_pose.resize(nj_torso); //Definition of Torso pose

    while(!encs_torso->getEncoders(encoders_torso.data()))
    {
            Time::delay(0.01);
            cout << "." << endl;
    }

    velocities_torso = 10;//Set torso velocity
    torso_pose = encoders_torso;
   
	// TO BE IMPLEMENTED
    return true;
}

bool VizzyTorso::performMotion(yarp::sig::Vector pos ) {
    bool done = false;
    pos_torso->setRefSpeeds(velocities_torso.data());
    pos_torso->positionMove(pos.data());
    Time::delay(2);
    while(!done) {
        pos_torso->checkMotionDone(&done);
        
        Time::delay(0.00001);   // Alterado
    }
    cout << "Pose torso" << pos.data() <<  endl;
    done = false;

    return done;
}

/**Update**/
bool VizzyTorso::updateModule() {
    Int16 *commandreceived;
//    if(commandPort.getInputCount()<=0)
//        return true;
    commandreceived = command_sub_torso.read(false);
    bool done = false;
    if (commandreceived!=NULL) {
        cout << "I received command torso" << commandreceived->data << endl;
        switch(commandreceived->data) {

            case 0: //Home position
                
                torso_pose = 0;
                done = performMotion(torso_pose);
                cout << "Home position torso" << endl;

                break;
            //For future implementation. In sync with the armRoutines switch 
            // case 1: 
            //     break;
            // case 2:
            //     break;
            // case 3:
            //     break;
            // case 4:
            //     break;

            // case 5:
            //     break;

            // case 6:
            //     break;

            // case 7:
            //     break;

            // case 8:
            //     break;

            // case 9:
            //     break;

            // case 10:
            //     break;

            // case 11: //fear
            //     break;

            // case 13:
            //     break;

            // case 14:
            //     break;

            // case 15:
            //     break;

            case 16: //singing
                
                torso_pose = 10;
                done = performMotion(torso_pose);

                Time::delay(3);
                cout << "Singing torso" << endl;
                
                break;

            case 18: //brushing
                torso_pose = 0;
                done = performMotion(torso_pose);
                cout << "Brushing torso" << endl;
                
                break;


            case 19: //Dancing
                //while(!){  } loop that waits
                torso_pose = 6;
                done = performMotion(torso_pose);
                cout << "Dancing pose" << endl;
                break;

            default:
                cout << "unknown command" << endl;
        }
    }
    

    return !_closing;

}

bool VizzyTorso::interruptModule() {
    cout << "Interrupting your torso module, for port cleanup" << endl;

    command_sub_torso.interrupt();
    return true;
}

bool VizzyTorso::close() {

    /* optional, close port explicitly */
    cout << "Calling close function torso\n";

    command_sub_torso.close();
    return true;
}
