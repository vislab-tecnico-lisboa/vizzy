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

public:
  /**********************************************************/
  bool configure(ResourceFinder &rf)
  {
    string remote = rf.find("remote").asString().c_str();
    string local = rf.find("local").asString().c_str();
    double tol = rf.find("tol").asDouble();
    double trajtime = rf.find("trajtime").asDouble();
    randomTarget = false;

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

    if (randomTarget)
      Rand::init();
    else
      attachTerminal();

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
    bool done = false;
    arm->checkMotionDone(&done);
    if (done)
    {
      if (randomTarget)
      {
        Vector xd(3);
        xd[0] = Rand::scalar(1.0, 2.5);
        xd[1] = Rand::scalar(0.0, 2.0);
        xd[2] = 0.0;

        cout << endl;
        cout << "Solving for: (" << xd.toString().c_str() << ")" << endl;
        arm->goToPositionSync(xd);
        Vector odhat, qdhat;
        arm->getDesired(xdhat, odhat, qdhat);
        cout << "Going to: (" << xdhat.toString().c_str() << ")" << endl;
        cout << "Solved Configuration: [" << qdhat.toString().c_str() << "]" << endl;
      }
    }
    else
    {
      //Vector x, o;
      //arm->getPose(x, o);
      //cout << "Running: (" << x.toString().c_str() << ");  distance to go: " << norm(xdhat - x) << endl;
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
  rf.setDefaultConfigFile("client_left_arm.ini");
  rf.configure(argc,argv);
  rf.setDefault("remote", "server");
  rf.setDefault("local", "client");
  //rf.configure("ICUB_ROOT", argc, argv);
  
  ClientModule client;
  return client.runModule(rf);
}


