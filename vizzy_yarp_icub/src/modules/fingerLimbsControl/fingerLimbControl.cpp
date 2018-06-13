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
#include "fingerLimbControl_IDL.h"

#include <iostream>
#include <iomanip>
#include <string>
#include "ForceReadingThread.h"
#include <TactSensor.h>
#include <TactSensorArray.h>
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

class ClientModule : public RFModule,
                     public fingerLimbControl_IDL
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
    yarp::os::Subscriber<TactSensorArray> force_sensor_port;
    yarp::sig::Vector command, encoders;
    ForceReadingThread *sensor_reading_thread;

    double finger_set[3];       // new force setpoints 
    double finger_force[3];         // current force values
    double joint_inc[3];
    double inc_max;                           // max joint increment
    double joint_max;                        // max joint value for fingers (min is 0)
    double force_error;                      // accepeted force error [N]
	bool make_control;
	
    Vector sensor_force;

    // Controling with a PID for each motor
    //PID pid_finger = PID(0.1, 20, -20, 3.4, 0.1, 0.5); //if they are different create the 3
    PID pid_finger = PID(0.1, 20, -20, 0.95, 0.01, 0.0); //if they are different create the 3

    yarp::os::RpcServer rpcServerPort;

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
        double part_speeds[11] = {20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0};
        pos->setRefSpeeds(part_speeds);
	std::cout << "initialization done" << std::endl;

        //double finger_set[3] = {6.1, 5.2, 3.6};       // force setpoints 
        finger_set[0] = 6.1; // new force setpoints 
        finger_set[1] = 5.2;
        finger_set[2] = 3.6;
        finger_force[0] = 0.0; // current force values
        finger_force[1] = 0.0;
        finger_force[2] = 0.0;
        joint_inc[0] = 0.0;
        joint_inc[1] = 0.0;
        joint_inc[2] = 0.0;

        inc_max = 10;                           // max joint increment
        joint_max = 180;                        // max joint value for fingers (min is 0)
        force_error = 0.5;                      // accepeted force error [N]
	make_control = 0;

        sensor_force.resize(11); // 11 active sensors 
	ictrl->setControlMode(8,VOCAB_CM_POSITION);
	ictrl->setControlMode(9,VOCAB_CM_POSITION);
	ictrl->setControlMode(10,VOCAB_CM_POSITION);
        rpcServerPort.open("/"+moduleName+"/rpc:i");
        attach(rpcServerPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        rpcServerPort.close();

        sensor_reading_thread->threadRelease();
        if (robotDevice.isValid())
            robotDevice.close();

        return true;
    }

    /**********************************************************/
    bool updateModule()
    {
        encs->getEncoders(encoders.data());

        for(int sensor_i = 0; sensor_i < 11; sensor_i++) {
            Vector sensor_comp(3); // x y z of each sensor
            sensor_reading_thread->get_force(sensor_i,sensor_comp);

	    if (sensor_i == 4){
    		//std::cout << "x: " << sensor_comp[0] << "y: " << sensor_comp[1] << "z: " << sensor_comp[2] << std::endl;
            sensor_force[sensor_i]= std::abs(sensor_comp[1])+std::abs(sensor_comp[2]); // the x component in this sensor is weird, check this
	    }
            else{   
                sensor_force[sensor_i]= std::abs(sensor_comp[0])+std::abs(sensor_comp[1])+std::abs(sensor_comp[2]);
            }
	    if (sensor_force[sensor_i]>10){    //just in case some sensor breaks during experiment
		sensor_force[sensor_i]=10;
	    }
	    if (sensor_force[sensor_i]<0){    //just in case some sensor breaks during experiment
		sensor_force[sensor_i]=0;
	    }
	            
        }

        finger_force[0] = sensor_force[0]+sensor_force[1]+sensor_force[2];
        finger_force[1] = sensor_force[3]+sensor_force[4]+sensor_force[5];  //ignoring the tip
        finger_force[2] = sensor_force[7]+sensor_force[8]+sensor_force[9]; // ignoring the tip     
      
        //std::cout << "Thumb Force: " << finger_force[0] << "Setpoint: " << finger_set[0] << std::endl;
        //std::cout << "Index Force: " << finger_force[1] << "Setpoint: " << finger_set[1] << std::endl;
        //std::cout << "Others Force: "<< finger_force[2] << "Setpoint: " << finger_set[2] << std::endl;
        std::cout << std::setprecision(3) << std::fixed;

        //std::cout << "Thumb: " << finger_force[0] << "Index: " << finger_force[1] << "Mid: " << finger_force[2] << std::endl;

        std::cout << "Thumb 1: " << sensor_force[0] << "2: " << sensor_force[1] << "3: " << sensor_force[2] << std::endl;
        std::cout << "Thumb - Force: " << finger_force[0]<< " Motor: " << encoders[8] << std::endl;

        std::cout << "Index 1: " << sensor_force[3] << "2: " << sensor_force[4] << "3: " << sensor_force[5] << " Total: " << finger_force[1]<< std::endl;
        std::cout << "Index - Force: " << finger_force[1]<< " Motor: " << encoders[9] << std::endl;

        //std::cout << "Mid 1: " << sensor_force[7] << "2: " << sensor_force[8] << "3: " << sensor_force[9] << std::endl;
        std::cout << "Mid - Force: " << finger_force[2]<< " Motor: " << encoders[10] << std::endl;


	if (make_control==1)
	{
		for (int finger_i = 0; finger_i < 3; finger_i++)
            {
                //condition for doing control or keeping
                if (std::abs(finger_force[finger_i] - finger_set[finger_i]) > force_error)
                {
                    double temp_pid = pid_finger.calculate(finger_set[finger_i], finger_force[finger_i]);

                    //relation factor between the force and the motor angle
                    joint_inc[finger_i] = temp_pid; // isto não dá para incluir no kp? deve dar pois

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
                    //std::cout << "Value: " << encoders[8+finger_i] << "Increment: " << joint_inc[finger_i] << std::endl;      
                }
                else
                {
                    joint_inc[finger_i]=0;
                    //std::cout << "Finger " << finger_i << " is ok!" << std::endl;
                }
            } // end for

            //change the enconders 8, 9 and 10 
            pos->positionMove(8,encoders[8]+joint_inc[0]);
            pos->positionMove(9,encoders[9]+joint_inc[1]);
            pos->positionMove(10,encoders[10]+joint_inc[2]);

	}

        return true;
    }

    /**********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    bool interruptModule(){
        rpcServerPort.interrupt();

        std::cout << "Interrupting the module" << std::endl;
        return true;
    }

    // IDL functions
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    bool grab(const std::string &type)
    {
        if (type=="one")
        {
		make_control=1; 
        }
        else if (type=="two")
        {
		make_control=0;
            // handshake constant
            const double grabTwo8 = 118;
            const double grabTwo9 = 105;
            const double grabTwo10 = 185;

            pos->positionMove(8, grabTwo8);
            pos->positionMove(9, grabTwo9);
            pos->positionMove(10, grabTwo10);
        }
        else
        {
            std::cout << "not implemented" << std::endl;
        }

        return true;
    }

    bool release()
    {
        // position pre-handshake
	make_control=0;
        const double release0 = -3.6;
        const double release1 = 73.5;
	const double release2 = 0.0;
        const double release3 = -8.5;
        const double release4 = 47.25;
	const double release5 = -10.2;
        const double release6 = 28;
        const double release7 = 18.2;
	const double release8 = 37.5;
        const double release9 = 45;
        const double release10 = 69.75;

        pos->positionMove(0, release0);
        pos->positionMove(1, release1);
	pos->positionMove(2, release2);
        pos->positionMove(3, release3);
        pos->positionMove(4, release4);
	pos->positionMove(5, release5);
        pos->positionMove(6, release6);
        pos->positionMove(7, release7);
	pos->positionMove(8, release8);
        pos->positionMove(9, release9);
        pos->positionMove(10, release10);

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
