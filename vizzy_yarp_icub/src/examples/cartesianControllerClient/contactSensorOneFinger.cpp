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
  double accumulated_distance;
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
    Bottle &grp=rf.findGroup("init_position");
    initial_position.resize(3);
    for (int i=0; i<3; i++){
    initial_position[i]=grp.get(1+i).asDouble();
    std::cout << "Initial position: " << initial_position[i] << std::endl;
    }
    initial_orientation.resize(4);
    Bottle &grp1=rf.findGroup("init_orienta");
    for (int i=0; i<4; i++){
    initial_orientation[i]=grp1.get(1+i).asDouble();
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

    // open the view
    client.view(arm);
    //arm->setTrajTime(trajtime);
    arm->setInTargetTol(tol);
    //double part_speeds[8] = {30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0};
    double part_speeds[8] = {12.0,12.0,12.0,12.0,12.0,12.0,12.0,12.0};
    ipos->setRefSpeeds(part_speeds);
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
         double *tmp = new double[n_joints];
         /*tmp[0]= 10.15;
         tmp[1]= 85.11;
         tmp[2] = 11.90;
         tmp[3]= -77.00;
         tmp[4] = 29.40;
         tmp[5]= -69.70;
         tmp[6]=-17.92;
         tmp[7]=-27.00;*/
	 /*tmp[0]= 0.0;
         tmp[1]= 0.0;
         tmp[2] = 40.0;
         tmp[3]= 0.0;
         tmp[4] = 50.0;
         tmp[5]= 0.0;
         tmp[6]= 0.0;
         tmp[7]=0.0;
         cout << "Before sending arm position!!" << endl;
         for (size_t j=0; j<8; j++){
             iMode2->setControlMode(j,VOCAB_CM_POSITION);
         }
         iMode2_torso->setControlMode(0,VOCAB_CM_POSITION);
         int my_joints[8] = {0,1,2,3,4,5,6,7};
         ipos->positionMove(tmp);
         cout << "Sent arm position!!" << endl;
         double *tmp1 = new double[1];
         tmp1[0] = 0.0;
         ipos_torso->positionMove(0,tmp1[0]);
         cout << "Sent torso position!!" << endl;
	 Time::delay(10);*/
	 /*bool done = false;
	  while(!done) {
             ipos->checkMotionDone(&done);
	     cout << "Not done!!" << endl;
             Time::delay(0.01);   // Alterado
          }*/
         arm->goToPoseSync(initial_position,initial_orientation);
         bool done = false;
         //arm->checkMotionDone(&done);
         /*while(!done) {
             arm->checkMotionDone(&done);
	     cout << "Not done!!" << endl;
             Time::delay(0.01);   // Alterado
         }*/
     double timeout = 5.0;
     done = arm->waitMotionDone(0.1,timeout);
     //arm->waitMotionDone(0.01);
     //Time::delay(5.0);
         cout << "Home position done!!" << endl;
         /*while (!connected_port){
           connected_port = Network::connect(sensor_remote_port, sensor_local_port);
           cout << "Waiting for port!!" << done << endl;
           Time::delay(1);
         }*/
         if(!done){
            yWarning("Something went wrong with the initial approach, using timeout");
            done = arm->waitMotionDone(0.1,timeout);
	    done = true;
         }
         if (done){
             current_state=1;
         }


     }
     else if (current_state==1){
         arm->getPose(initial_position,initial_orientation);
         cout << initial_position[0] << " y: " << initial_position[1] << " z:" << initial_position[2] << endl;
         cout << " or1: " << initial_orientation[0] << " or2: "<< initial_orientation[1] << " or3: " <<initial_orientation[2] << " angle:" << initial_orientation[3]<< endl;
         Bottle* readingSensor = sensor_readings.read(false);
         accumulated_distance = 0.0;
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
         /*arm->getPose(initial_position,initial_orientation);
         Vector new_position(3);
         new_position = initial_position;
         //new_position[0]-=0.03;
         new_position[1]-=0.01;
         cout << "new x: " <<new_position[0] << " y: " << new_position[1] << " z:" << new_position[2] << endl;
         for (size_t j=0; j<8; j++){
             iMode2->setControlMode(j,VOCAB_CM_POSITION_DIRECT);
         }
         iMode2_torso->setControlMode(0,VOCAB_CM_POSITION_DIRECT);
         arm->goToPoseSync(new_position,initial_orientation);
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
             cout << current_position[0] << " y: " << current_position[1] << " z:" << current_position[2] << endl;
             cout << " or1: " << current_orientation[0] << " or2: "<< current_orientation[1] << " or3: " <<current_orientation[2] << " angle:" << current_orientation[3]<< endl;
             Vector new_position(3);
             new_position = current_position;
             new_position[1]-=0.02;
	     //new_position[2]+=0.005;
             cout << "new x: " <<new_position[0] << " y: " << new_position[1] << " z:" << new_position[2] << endl;
             /*for (size_t j=0; j<8; j++){
                 iMode2->setControlMode(j,VOCAB_CM_POSITION_DIRECT);
             }
             iMode2_torso->setControlMode(0,VOCAB_CM_POSITION_DIRECT);*/





             //arm->goToPoseSync(new_position,current_orientation);
             double timeHere;
             arm->getTrajTime(&timeHere);
             arm->setTrajTime(trajtime);
             Vector xdot(3); // move the end-effector along x-axis at specified velocity
             xdot[0] = 0.0;    // 0.09 [m/s]
             xdot[1] = -0.02;
             xdot[2] = 0.0;
             Vector odot(4); // no rotation is required
             odot=0.0; // [rad/s]
             arm->setTaskVelocities(cartesian_velocity,odot);

             yDebug("waiting 2.5 seconds");
             Time::delay(trajtime*1.5);
             arm->stopControl();
             arm->setTrajTime(timeHere);
             Vector xdhat,odhat, qdhat;

             arm->getDesired(xdhat, odhat, qdhat);
             accumulated_distance=initial_position[1]-xdhat[1];
             cout << "Going to: (" << xdhat.toString().c_str() << ")" << endl;
             cout << "Solved Configuration: [" << qdhat.toString().c_str() << "]" << endl;
             cout << "Accumulated distance: " << accumulated_distance << endl;
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


