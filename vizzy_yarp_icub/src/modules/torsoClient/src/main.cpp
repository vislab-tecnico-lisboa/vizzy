// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include "vizzyTorso.h"

using namespace yarp::os;
using namespace std;

int main(int argc, char *argv[]) {

    Network yarp;
    bool value;

    if(! yarp.checkNetwork() ) {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return 1;
    }
    yarp::os::Node node("/yarp/vizzyTorsoInterface");

    VizzyTorso module;
    ResourceFinder rf; 
    rf.setVerbose(true);
    rf.setDefaultConfigFile("vizzyTorsoInterface.ini");
    value = rf.configure(argc, argv);
    cout <<"Cheguei aqui" << value <<  endl;
    module.runModule(rf);
    return 0;
}
