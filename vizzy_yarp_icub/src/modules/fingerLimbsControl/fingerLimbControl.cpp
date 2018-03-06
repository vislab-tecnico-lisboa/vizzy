/* 
 * Copyright (C) 2017 Computer and Robot Vision Lab - Instituto Superior Tecnico
 * Author: Plinio Moreno
 * email:  plinio@isr.tecnico.ulisboa.pt
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iostream>
#include <iomanip>
#include <string>
#include "ForceReadingThread.h"
#include <vizzy_tactile_TactSensor.h>
#include <vizzy_tactile_TactSensorArray.h>
#include <yarp/os/Subscriber.h>
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/**
 * This class implements the client.
 */

class ClientModule : public RFModule
{
protected:
    string moduleName;
    string robotName;
    string armName;
    Property options;
    PolyDriver robotDevice;
    IPositionControl *pos;
    IEncoders *encs;
    IControlMode2 *ictrl;

    VectorOf<int> jntArm;
    yarp::os::Subscriber<vizzy_tactile_TactSensorArray> force_sensor_port;
    yarp::sig::Vector command, encoders;
    ForceReadingThread *sensor_reading_thread;
public:
    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {
        moduleName = rf.check("name", Value("vizzyArmRoutines"),
                              "Module name (string)").asString();
        robotName = rf.check("robot", Value("vizzy"),"Robot name (string)").asString();
            armName = rf.check("arm", Value("right"),"Arm name (string)").asString();
        setName(moduleName.c_str());

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
            return false;
        }
        while (!force_sensor_port.topic("/tactileForceField")) {
              cerr<< "Failed to connect to subscriber to /tactileForceField\n";
              Time::delay(0.01);
        }
        //ForceReadingThread sensor_reading_thread(&force_sensor_port);
        sensor_reading_thread = new ForceReadingThread(&force_sensor_port);
        int nj=0;
        pos->getAxes(&nj);
        encoders.resize(nj);
        return true;
    }

    /**********************************************************/
    bool close()
    {
        sensor_reading_thread->threadRelease();
        if (robotDevice.isValid())
            robotDevice.close();

        return true;
    }

    /**********************************************************/
    bool updateModule()
    {
        encs->getEncoders(encoders.data());
        Vector index_finger_force(3);
        sensor_reading_thread->get_force(index_finger_force);
        std::cout << "Fz: " << index_finger_force[2]<< std::endl;
        if (index_finger_force[2]>1.0)
            pos->positionMove(8,encoders[8]+5.0);
        return true;
    }

    /**********************************************************/
    double getPeriod()
    {
        return 0.8;
    }

    bool interruptModule(){
        std::cout << "Closing the module" << std::endl;
        return true;
    }
};

/**********************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout << "Error: yarp server does not seem available" << endl;
        return -1;
    }
    yarp::os::Node node("/yarp/fingerLimbsControl");
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("vizzyFingerLimbControl");
    rf.setDefaultConfigFile("finger_control.ini");
    rf.configure(argc,argv);
    rf.setDefault("remote", "server");
    rf.setDefault("local", "client");
    //rf.configure("ICUB_ROOT", argc, argv);

    ClientModule client;
    return client.runModule(rf);
}


