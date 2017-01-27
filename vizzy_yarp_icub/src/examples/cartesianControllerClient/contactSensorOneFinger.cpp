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
  double distance;
  double contact_tol;
  Vector control_reference;
  int current_state;
  bool connected_port;
  BufferedPort<Bottle> sensor_readings;
  string sensor_remote_port;
  string sensor_local_port;
  Vector current_position;
  Vector current_orientation;
public:
  /**********************************************************/
  bool configure(ResourceFinder &rf)
  {
    string remote = rf.find("remote").asString().c_str();
    string local = rf.find("local").asString().c_str();
    sensor_remote_port = rf.find("sensRemotPort").asString().c_str();
    sensor_local_port = rf.find("sensLocalPort").asString().c_str();
    double tol = rf.find("tol").asDouble();
    double trajtime = rf.find("trajtime").asDouble();
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
    distance = rf.find("distance").asDouble();
    contact_tol = rf.find("contact_tol").asDouble();
    randomTarget = false;
    printf("Distance to be pushed is %f mts and contact tolerance is %f.\n",distance,contact_tol);
    printf("Trajectory time is %f and target tolerance is %f.\n",trajtime,tol);

    // Just usual things...
    Property option("(device cartesiancontrollerclient)");
    option.put("remote", ("/" + remote).c_str());
    option.put("local", ("/" + local).c_str());

    if (!client.open(option))
      return false;

    // open the view
    client.view(arm);
    arm->setTrajTime(trajtime);
    arm->setInTargetTol(tol);

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
         arm->goToPoseSync(initial_position,initial_orientation);
         bool done = false;
         arm->checkMotionDone(&done);
         if (done){
             current_state=1;
         }
         while (!connected_port){
           connected_port = Network::connect(sensor_remote_port, sensor_local_port);
           Time::delay(1);
         }
     }
     else if (current_state==1){
         Bottle* readingSensor = sensor_readings.read(false);
         if (readingSensor != NULL){
             double deltaZ = readingSensor->get(2).asDouble()-readingSensor->get(5).asDouble();
             double deltaY = readingSensor->get(1).asDouble()-readingSensor->get(4).asDouble();
             double deltaX = readingSensor->get(0).asDouble()-readingSensor->get(3).asDouble();
             double total_displacement = sqrt(deltaZ*deltaZ+deltaY*deltaY+deltaX*deltaX);
             if (total_displacement < contact_tol){
                 cout << "The sensor is not toching any object" << endl;
             }
             else{
                 current_state = 2;
             }
         }
     }
     else if (current_state==2){
         Bottle* readingSensor = sensor_readings.read();
         //while(readingSensor==NULL)
         //    readingSensor = sensor_readings.read(false);
         double deltaZ = control_reference[2]-readingSensor->get(5).asDouble();
         double deltaY = control_reference[1]-readingSensor->get(4).asDouble();
         double deltaX = control_reference[0]-readingSensor->get(3).asDouble();
         double error = sqrt(deltaZ*deltaZ+deltaY*deltaY+deltaX*deltaX);
         if (deltaZ<0 && error > contact_tol){
             //move the arm the delta value forward
             arm->getPose(current_position,current_orientation);
             Vector new_position(3);
             new_position[1]+=deltaZ*(1.3);
             arm->goToPoseSync(new_position,current_orientation);
         }
         else if (deltaZ<0 && error < contact_tol){
             // do not move the arm
             cout << "Force goal reached!" << endl;
             cout << "Waiting for three seconds and then moving to the initial pose" << endl;
             Time::delay(1);
             current_state=0;
         }
         else if (deltaZ >0 && error > contact_tol){
             //move the arm backwards the delta value
             arm->getPose(current_position,current_orientation);
             Vector new_position(3);
             new_position[1]-=deltaZ*(1.3);
             arm->goToPoseSync(new_position,current_orientation);
         }

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


