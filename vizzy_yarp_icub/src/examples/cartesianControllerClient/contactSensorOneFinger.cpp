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
    PolyDriver client;
    ICartesianControl *arm;
    Vector xdhat;
    bool randomTarget;
    Vector home_position;
    Vector home_orientation;
    Vector initial_position;
    Vector initial_orientation;
    Vector cartesian_velocity;
    double distance;
    double contact_tol;
    Vector control_reference;
    double trajtime;
    int current_state;
    bool connected_port;
    BufferedPort<Bottle> sensor_readings;
    string sensor_remote_port;
    string sensor_local_port;
    Vector current_position;
    Vector current_orientation;
    IPositionControl *ipos;
    IPositionControl *ipos_torso;
    PolyDriver dd;
    PolyDriver dd_torso;
    int n_joints;
    IControlMode2 *iMode2;
    IControlMode2 *iMode2_torso;
    double accumulated_distance_x;
    double accumulated_distance_y;
    double accumulated_distance_z;
    double accumulated_distance;
    VectorOf<int> jntArm;
    yarp::os::Subscriber<vizzy_tactile_TactSensorArray> force_sensor_port;
    ForceReadingThread sensor_reading_thread;
public:
    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {
        Vector oy(4), oz(4);
        oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=+M_PI;
        oz[0]=1.0; oz[1]=0.0; oz[2]=0.0; oz[3]=-M_PI/2.0;
        Matrix Ry=yarp::math::axis2dcm(oy);        // from axis/angle to rotation matrix notation
        Matrix Rz=yarp::math::axis2dcm(oz);
        Matrix R=Rz*Ry;                            // compose the two rotations keeping the order
        Vector o=yarp::math::dcm2axis(R);
        std::cout << "orientation for tapping : " << o.toString() << std::endl;

        Vector oy1(4);
        oy1[0]=0.0; oy1[1]=0.0; oy1[2]=1.0; oy1[3]=-M_PI;
        Matrix Ry1=yarp::math::axis2dcm(oy1);        // from axis/angle to rotation matrix notation
        Matrix R1=Ry1;                            // compose the two rotations keeping the order
        Vector o1=yarp::math::dcm2axis(R1);
        std::cout << "orientation for pushing : " << o1.toString() << std::endl;

        string remote = rf.find("remote").asString().c_str();
        string local = rf.find("local").asString().c_str();
        sensor_remote_port = rf.find("sensRemotPort").asString().c_str();
        sensor_local_port = rf.find("sensLocalPort").asString().c_str();
        double tol = rf.find("tol").asDouble();
        trajtime = rf.find("trajtime").asDouble();
        Bottle &grp=rf.findGroup("home_position");
        home_position.resize(3);
        for (int i=0; i<3; i++){
            home_position[i]=grp.get(1+i).asDouble();
            std::cout << "Home position: " << home_position[i] << std::endl;
        }
        home_orientation.resize(4);
        Bottle &grp1=rf.findGroup("home_orientation");
        for (int i=0; i<4; i++){
            home_orientation[i]=grp1.get(1+i).asDouble();
            std::cout << "Home orientation: " << home_orientation[i] << std::endl;
        }
        Bottle &grp4=rf.findGroup("initial_position");
        initial_position.resize(3);
        for (int i=0; i<3; i++){
            initial_position[i]=grp4.get(1+i).asDouble();
            std::cout << "Initial position: " << initial_position[i] << std::endl;
        }
        initial_orientation.resize(4);
        Bottle &grp5=rf.findGroup("initial_orientation");
        for (int i=0; i<4; i++){
            initial_orientation[i]=grp5.get(1+i).asDouble();
            std::cout << "Initial orientation: " << initial_orientation[i] << std::endl;
        }
        control_reference.resize(3);
        Bottle &grp2=rf.findGroup("control_ref");
        for (int i=0; i<3; i++){
            control_reference[i]=grp2.get(1+i).asDouble();
            std::cout << "Control reference: " << control_reference[i] << std::endl;
        }
        cartesian_velocity.resize(3);
        Bottle &grp3=rf.findGroup("cart_velocity");
        for (int i=0; i<3; i++){
            cartesian_velocity[i]=grp3.get(1+i).asDouble();
            std::cout << "Cartesian velocity: " << cartesian_velocity[i] << std::endl;
        }
        distance = rf.find("distance").asDouble();
        contact_tol = rf.find("contact_tol").asDouble();
        randomTarget = false;
        printf("Distance to be pushed is %f mts and contact tolerance is %f.\n",distance,contact_tol);
        printf("Trajectory time is %f and target tolerance is %f.\n",trajtime,tol);

        string robot=rf.find("robot").asString().c_str();
        string part=rf.find("part").asString().c_str();
        n_joints=rf.find("num_joints").asInt();
        // Just usual things...
        Property option("(device cartesiancontrollerclient)");
        option.put("remote", ("/" + remote).c_str());
        option.put("local", ("/" + local).c_str());

        Property options;
        options.put("robot", robot);//Needs to be read from a config file
        options.put("device", "remote_controlboard");
        options.put("remote", "/" + robot+"/"+ part );
        options.put("local", "/" + robot+"/"+ part + "/_pos_interface");
        options.put("part", part);


        Property options_torso;
        options_torso.put("robot", robot);//Needs to be read from a config file
        options_torso.put("device", "remote_controlboard");
        options_torso.put("remote", "/" + robot+"/"+ "torso" );
        options_torso.put("local", "/" + robot+"/"+ "torso" + "/_pos_interface");
        options_torso.put("part", "torso");

        dd.open(options);
        cout << "Arm driver done!!" << endl;
        dd_torso.open(options_torso);
        cout << "Torso driver done!!" << endl;
        bool ok;
        ok = dd.view(ipos);
        ok &= dd.view(iMode2);
        ok &= dd_torso.view(ipos_torso);
        ok &= dd_torso.view(iMode2_torso);
        if (!client.open(option) || !ok)
            return false;

        while (!force_sensor_port.topic("/tactileForceField")) {
              cerr<< "Failed to connect to subscriber to /tactileForceField\n";
              Time::delay(0.01);
          }
        sensor_reading_thread = ForceReadingThread(&force_sensor_port);
        // open the view
        client.view(arm);
        //arm->setTrajTime(trajtime);
        arm->setInTargetTol(tol);
        //double part_speeds[8] = {30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0};
        double part_speeds[8] = {12.0,12.0,12.0,12.0,12.0,12.0,12.0,12.0};
        ipos->setRefSpeeds(part_speeds);
        for (int i=0; i<8; i++)
            jntArm.push_back(i);
        Vector dof;
        arm->getDOF(dof);
        for (int i = 0; i < dof.length(); i++)
        {
            double min, max;
            arm->getLimits(i, &min, &max);

            // these margins are just to prevent the
            // solver from solving for exactly the
            // joints bounds
            min += 1.0;
            max -= 1.0; // [deg]
            arm->setLimits(i, min, max);
        }
        bool opened_reading_port = sensor_readings.open(sensor_local_port);
        if (!opened_reading_port)
            return false;
        if (randomTarget)
            Rand::init();
        else
            attachTerminal();
        current_state=0;
        connected_port=false;
        current_position.resize(3);
        current_orientation.resize(4);
        cout << "Init done!!" << endl;
        return true;
    }

    /**********************************************************/
    bool close()
    {
        if (client.isValid())
            client.close();

        return true;
    }

    /**********************************************************/
    bool updateModule()
    {
        if(current_state==0){
            double timeout = 4.0;
	    accumulated_distance=0.0;
            double *tmp = new double[n_joints];
            //--
            //BEGIN Setting the motor control in POSITION mode for each joint
            //--
            VectorOf<int> modes;
            modes.resize(8,VOCAB_CM_POSITION);
            iMode2->setControlModes(jntArm.size(),jntArm.getFirst(),modes.getFirst());
            //--
            // END Setting the motor control in POSITION mode for each joint
            //--

            //--
            //BEGIN Setting the motor angular positions for each joint
            //--
            //Initial values left arm joints
            double joints_arm[8] = {-12,30,12,-22,66,-39,23,0.0};
            ipos->positionMove(joints_arm);
            bool motionDone_arm=false;
	    double init_time=Time::now();
	    double current_time;
            while (motionDone_arm==false && current_time-init_time<timeout*2.0){
                ipos->checkMotionDone(&motionDone_arm);
		current_time = Time::now();
            }
            //--
            //BEGIN Setting the motor control in POSITION_DIRECT mode for each joint
            //--
            //VectorOf<int> modes;
            modes.resize(8,VOCAB_CM_POSITION_DIRECT);
            iMode2->setControlModes(jntArm.size(),jntArm.getFirst(),modes.getFirst());
            //--
            // END Setting the motor control in POSITION_DIRECT mode for each joint
            //--

            /*drvTorso.close();
            drvArm.close();
            jntArm.clear();
            jntTorso.clear();*/
            //--
            // END Setting the motor angular positions for each joint
            //--

            //--
            //BEGIN Setting the home position in cartesian task space
            //--
            /*arm->goToPoseSync(home_position,home_orientation);
            bool done = false;
            timeout = 5.0;
            done = arm->waitMotionDone(0.1,timeout);
            cout << "Home position done!!" << endl;
            if(!done){
                yWarning("Something went wrong with the initial approach, using timeout");
                done = arm->waitMotionDone(0.1,timeout);
                done = true;
            }*/
            //--
            //END Setting the home position in cartesian task space
            //--
            arm->goToPoseSync(initial_position,initial_orientation);
            bool done = false;
            done = arm->waitMotionDone(0.1,timeout);
            cout << "Initial position done!!" << endl;
            if(!done){
                yWarning("Something went wrong with the initial approach, using timeout");
                done = arm->waitMotionDone(0.1,timeout);
                done = true;
            }
            arm->getPose(home_position,home_orientation);
            cout << "Initial position: " << home_position[0] << " y: " << home_position[1] << " z:" << home_position[2] << endl;
            cout << "Initial orientation: or1: " << home_orientation[0] << " or2: "<< home_orientation[1] << " or3: " <<home_orientation[2] << " angle:" << home_orientation[3]<< endl;

            if (done){
                current_state=1;
            }


        }
        else if (current_state==1){
            arm->getPose(home_position,home_orientation);
            //cout << home_position[0] << " y: " << home_position[1] << " z:" << home_position[2] << endl;
            //cout << " or1: " << home_orientation[0] << " or2: "<< home_orientation[1] << " or3: " <<home_orientation[2] << " angle:" << home_orientation[3]<< endl;
            Bottle* readingSensor = sensor_readings.read(false);
            accumulated_distance_x=accumulated_distance_y=accumulated_distance_z = 0.0;
            current_state=2;
            if (readingSensor != NULL){
                double deltaZ = readingSensor->get(2).asDouble()-readingSensor->get(5).asDouble();
                double deltaY = readingSensor->get(1).asDouble()-readingSensor->get(4).asDouble();
                double deltaX = readingSensor->get(0).asDouble()-readingSensor->get(3).asDouble();
                double total_displacement = sqrt(deltaZ*deltaZ+deltaY*deltaY+deltaX*deltaX);
                if (total_displacement < contact_tol){
                    cout << "The sensor is not touching any object" << endl;
                }
                else{
                    current_state = 2;
                }
            }
            /*arm->getPose(home_position,home_orientation);
         Vector new_position(3);
         new_position = home_position;
         //new_position[0]-=0.03;
         new_position[1]-=0.01;
         cout << "new x: " <<new_position[0] << " y: " << new_position[1] << " z:" << new_position[2] << endl;
         for (size_t j=0; j<8; j++){
             iMode2->setControlMode(j,VOCAB_CM_POSITION_DIRECT);
         }
         iMode2_torso->setControlMode(0,VOCAB_CM_POSITION_DIRECT);
         arm->goToPoseSync(new_position,home_orientation);
         Vector xdhat,odhat, qdhat;
         arm->getDesired(xdhat, odhat, qdhat);
         cout << "Going to: (" << xdhat.toString().c_str() << ")" << endl;
         cout << "Solved Configuration: [" << qdhat.toString().c_str() << "]" << endl;
         Time::delay(10);*/
        }
        else if (current_state==2){
            /*Bottle* readingSensor = sensor_readings.read();
         //while(readingSensor==NULL)
         //    readingSensor = sensor_readings.read(false);
         double deltaZ = control_reference[2]-readingSensor->get(5).asDouble();
         double deltaY = control_reference[1]-readingSensor->get(4).asDouble();
         double deltaX = control_reference[0]-readingSensor->get(3).asDouble();
         double error = sqrt(deltaZ*deltaZ+deltaY*deltaY+deltaX*deltaX);*/
            double error=contact_tol+0.01;
            double deltaZ=-0.01;
            if (deltaZ<0 && error > contact_tol && accumulated_distance<0.1){
                //move the arm the delta value forward
                arm->getPose(current_position,current_orientation);
                //cout << current_position[0] << " y: " << current_position[1] << " z:" << current_position[2] << endl;
                //cout << " or1: " << current_orientation[0] << " or2: "<< current_orientation[1] << " or3: " <<current_orientation[2] << " angle:" << current_orientation[3]<< endl;
                Vector new_position(3);
                new_position = current_position;
                new_position[1]-=0.02;
                //new_position[2]+=0.005;
                //cout << "new x: " <<new_position[0] << " y: " << new_position[1] << " z:" << new_position[2] << endl;
                /*for (size_t j=0; j<8; j++){
                 iMode2->setControlMode(j,VOCAB_CM_POSITION_DIRECT);
             }
             iMode2_torso->setControlMode(0,VOCAB_CM_POSITION_DIRECT);*/





                //arm->goToPoseSync(new_position,current_orientation);
                double timeHere;
                arm->getTrajTime(&timeHere);
                arm->setTrajTime(trajtime);
                Vector odot(4); // no rotation is required
                odot=0.0; // [rad/s]
                arm->setTaskVelocities(cartesian_velocity,odot);

                yDebug("waiting 2.5 seconds");
                //Time::delay(trajtime);



		double init_time=Time::now();
	    	double current_time;
		Vector xdhat,odhat, qdhat;
        Vector index_finger_force(3);
                while (current_time-init_time<trajtime){
                    //arm->getDesired(xdhat, odhat, qdhat);
            //cout << "While waiting going to: (" << xdhat.toString().c_str() << ")" << endl;
            //current_time = Time::now();
                    sensor_reading_thread.get_force(index_finger_force);
                    std::cout << "Fz: " << index_finger_force[2] << std::endl;
                }
                arm->stopControl();
                arm->setTrajTime(timeHere);

                arm->getDesired(xdhat, odhat, qdhat);
                accumulated_distance_y=home_position[1]-xdhat[1];
                accumulated_distance_x=home_position[0]-xdhat[0];
                accumulated_distance_z=home_position[2]-xdhat[2];
                accumulated_distance+=sqrt(accumulated_distance_y*accumulated_distance_y+
                                           accumulated_distance_x*accumulated_distance_x+accumulated_distance_z*accumulated_distance_z);
                cout << "Going to: (" << xdhat.toString().c_str() << ")" << endl;
                cout << "Solved Configuration: [" << qdhat.toString().c_str() << "]" << endl;
                cout << "Accumulated distance x: " << accumulated_distance_x << endl;
                cout << "Accumulated distance y: " << accumulated_distance_y << endl;
                cout << "Accumulated distance z: " << accumulated_distance_z << endl;
                Time::delay(1);
                //new_position[1]+=deltaZ*(1.3);
                //arm->goToPoseSync(new_position,current_orientation);
            }
            else if (deltaZ<0 && error > contact_tol && accumulated_distance>=0.1){
                // do not move the arm
                cout << "Push object task finished successfully!" << endl;
                cout << "Waiting for three seconds and then moving to the initial pose" << endl;
                Time::delay(3);
                current_state=0;
            }
            /*else if (deltaZ >0 && error > contact_tol){
             //move the arm backwards the delta value
             arm->getPose(current_position,current_orientation);
             Vector new_position(3);
             new_position[1]-=deltaZ*(1.3);
             arm->goToPoseSync(new_position,current_orientation);
         }*/

        }
        return true;
    }

    /**********************************************************/
    double getPeriod()
    {
        return 0.4;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        //set pos (0.0 0.33 0.6)
        if (command.get(0).isString() && (command.get(0).asString() == "help")){
            printf("to set position in meters use \"set pos (0.0 0.33 0.6)\".\n");
        }
        if (command.get(0).isString() && (command.get(0).asString() == "set"))
        {
            if (command.get(1).isString() && (command.get(1).asString() == "pos" && command.get(2).isList()))
            {
                Bottle *pos;
                pos = command.get(2).asList();
                bool done =true;
                //arm->checkMotionDone(&done);
                if (done)
                {
                    Vector xd(3);
                    xd[0] = pos->get(0).asDouble();
                    xd[1] = pos->get(1).asDouble();
                    xd[2] = pos->get(2).asDouble();

                    cout << endl;
                    cout << "Solving for: (" << xd.toString().c_str() << ")" << endl;
                    //arm->goToPositionSync(xd);
                    arm->goToPosition(xd);
                    Vector odhat, qdhat;
                    arm->getDesired(xdhat, odhat, qdhat);
                    cout << "Going to: (" << xdhat.toString().c_str() << ")" << endl;
                    cout << "Solved Configuration: [" << qdhat.toString().c_str() << "]" << endl;
                }
            }
        }
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
    yarp::os::Node node("/yarp/contactSensorControl");
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("vizzyCartesianControllerClient");
    rf.setDefaultConfigFile("client_right_arm_contact_sensor.ini");
    rf.configure(argc,argv);
    rf.setDefault("remote", "server");
    rf.setDefault("local", "client");
    //rf.configure("ICUB_ROOT", argc, argv);

    ClientModule client;
    return client.runModule(rf);
}


