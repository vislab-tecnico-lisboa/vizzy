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

#include "pid.h"
#include <cmath> 

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
	while (!force_sensor_port.topic("/tactileForceField")) {
              cerr<< "Failed to connect to subscriber to /tactileForceField\n";
              Time::delay(0.01);
        }
        //ForceReadingThread sensor_reading_thread(&force_sensor_port);
        sensor_reading_thread = new ForceReadingThread(&force_sensor_port);
        bool ok;
        ok = robotDevice.view(pos);
        ok = ok && robotDevice.view(encs);
        ok = ok && robotDevice.view(ictrl);
	ok&=sensor_reading_thread->start();
        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return false;
        }
        
        int nj=0;
        pos->getAxes(&nj);
        encoders.resize(nj);
        double part_speeds[11] = {10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,20.0,20.0,20.0};
        pos->setRefSpeeds(part_speeds);
	std::cout << "initialization done" << std::endl;
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

	    //std::cout << "Before reading force" << std::endl;
        Vector sensor_force(11); // 11 active sensors 

        for(int sensor_i = 0; sensor_i < 11; sensor_i++) {
            Vector sensor_comp(3); // x y z of each sensor
            sensor_reading_thread->get_force(sensor_i,sensor_comp);
            sensor_force[sensor_i]= std::abs(sensor_comp[0])+std::abs(sensor_comp[1])+std::abs(sensor_comp[2]);           
        }

        // Controling with a PID for each motor

        PID pid_finger = PID(0.1, 20, -20, 0.4, 0.01, 0.4); //if they are different create the 3

        double finger_set[3] = {6, 5.4, 3.4};       // force setpoints 
        double finger_force[3] = {0, 0, 0};         // current force values
        double joint_inc[3] = {0, 0, 0};

        double inc_max = 10;                           // max joint increment
        double joint_max = 250;                        // max joint value for fingers (min is 0)
        double force_error = 0.5;                      // accepeted force error [N]

        finger_force[0] = sensor_force[0]+sensor_force[1]+sensor_force[2];
        finger_force[1] = sensor_force[3]+sensor_force[4]+sensor_force[5];  //ignoring the tip
        finger_force[2] = sensor_force[7]+sensor_force[8]+sensor_force[9]+sensor_force[10];      
      
        std::cout << "Thumb Force: " << finger_force[0] << "Setpoint: " << finger_set[0] << std::endl;
        std::cout << "Index Force: " << finger_force[1] << "Setpoint: " << finger_set[1] << std::endl;
        std::cout << "Others Force: "<< finger_force[2] << "Setpoint: " << finger_set[2] << std::endl;

        for (int finger_i = 0; finger_i < 3; finger_i++) {

            //condition for doing control or keeping
            if (std::abs(finger_force[finger_i] - finger_set[finger_i]) > force_error) {
                        
                double temp_pid = pid_finger.calculate(finger_set[finger_i], finger_force[finger_i]);

                //relation factor between the force and the motor angle
                joint_inc[finger_i] = temp_pid * 10; // isto não dá para incluir no kp? deve dar pois

                //conditions for incrmentation - No increment larger than inc_max
                if (std::abs(joint_inc[finger_i]) > inc_max){
                    joint_inc[finger_i] = inc_max*(joint_inc[finger_i]/std::abs(joint_inc[finger_i]));
                }

                //conditions for joint: 0 < angles < joint_max
                if (encoders[8+finger_i] + joint_inc[finger_i] > joint_max){
                    joint_inc[finger_i] = joint_max-encoders[8+finger_i]; 
                }
                if (encoders[8+finger_i] + joint_inc[finger_i] < 0){
                    joint_inc[finger_i] = 0 - encoders[8+finger_i];
                }
                std::cout << "Value: " << encoders[8+finger_i] << "Increment: " << joint_inc[finger_i] << std::endl;      
            }

            else {
                joint_inc[finger_i]=0;
                std::cout << "Finger " << finger_i << " is ok!" << std::endl;
            }        
        }

        //change to enconders 8, 9 and 10  
        //pos->positionMove(8,encoders[8]+joint_inc[0]);
        //pos->positionMove(9,encoders[9]+joint_inc[1]);
        //pos->positionMove(10,encoders[10]+joint_inc[2]);

        return true;
    }

    /**********************************************************/
    double getPeriod()
    {
        return 0.01;
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
    rf.setDefaultConfigFile("vizzyConfig_rightArm.ini");
    rf.configure(argc,argv);
    rf.setDefault("remote", "server");
    rf.setDefault("local", "client");
    //rf.configure("ICUB_ROOT", argc, argv);

    ClientModule client;
    return client.runModule(rf);
}


