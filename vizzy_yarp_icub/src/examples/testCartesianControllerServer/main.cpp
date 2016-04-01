/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

/**
 * This class launches the server.
 */
class ServerModule : public RFModule
{
protected:
  class SolverProcessor : public TypedReaderCallback<Bottle>
  {
  private:
    ServerModule *_testServer;
  public:
    SolverProcessor(ServerModule *testServer) :
      TypedReaderCallback<Bottle> ()
    {
      _testServer = testServer;
    }

    void onRead(Bottle& b)
    {
      printf("CALLBACK: \n");
      Bottle *xdBottle = b.get(0).asList()->get(1).asList();
      Bottle *xBottle = b.get(1).asList()->get(1).asList();
      Bottle *qBottle = b.get(2).asList()->get(1).asList();
      printf("\t[XD]\t%s\n", xdBottle->toString().c_str());
      printf("\t[X]\t%s\n", xBottle->toString().c_str());
      printf("\tJOINTS:\n");
      printf("\t\t[Q]\t%s\n", qBottle->toString().c_str());

      //_testServer->moveTorso(qBottle->get(0).asDouble());//DISABLED
      // _testServer->moveShoulder(qBottle->get(0).asDouble(), qBottle->get(1).asDouble(), qBottle->get(2).asDouble());
      _testServer->moveArm(qBottle->get(0).asDouble(), qBottle->get(1).asDouble(), qBottle->get(2).asDouble(), qBottle->get(3).asDouble(), qBottle->get(4).asDouble(), qBottle->get(5).asDouble(),
                           qBottle->get(6).asDouble());
    }
  };
  SolverProcessor *solverCallback;
  BufferedPort<Bottle> solverPort;
  PolyDriver partDrvTorso, partDrvArm;
  IPositionControl *posTorso, *posArm;
  IEncoders *encTorso, *encArm;
  bool movingTorso, movingArm;
  PolyDriver server;

private:
  void partMove(IPositionControl *posPart, double Count, ...)
  {
    double tempRef;
    va_list Numbers;
    va_start(Numbers, Count);
    for (int i = 0; i < Count; ++i)
    {
      tempRef = va_arg(Numbers, double);
      printf("\t%.2f", tempRef);
      posPart->positionMove(i, tempRef);
    }
    va_end(Numbers);
  }

  Property newPartProperties(string robot, string srvName, string part)
  {
    Property optPart;
    optPart.put("device", "remote_controlboard");
    optPart.put("remote", ("/" + robot + "/" + part).c_str());
    optPart.put("local", ("/" + srvName + "/" + part).c_str());
    optPart.put("robot", robot.c_str());
    optPart.put("part", part.c_str());
    return optPart;
  }

public:
  void moveArm(double joint0, double joint1, double joint2, double joint3, double joint4, double joint5, double joint6)
  {
    movingArm = true;
    printf("ARM:");
    partMove(posArm, 7, joint0, joint1, joint2, joint3, joint4, joint5, joint6);
    printf("\n");
  }
  void moveTorso(double joint0)
  {
    movingTorso = true;
    partMove(posTorso, 1, joint0);
  }
  /**********************************************************/
  bool configure(ResourceFinder &rf)
  {
    movingTorso = false;
    movingArm = false;
    // grab parameters from the congiguration file
    string robot = rf.find("robot").asString().c_str();
    string local = rf.find("local").asString().c_str();
    string solverName = rf.find("SolverNameToConnect").asString().c_str();
    string localSolverName = "/" + local + "/" + solverName + "/in";
    string solverNameOut = "/" + solverName + "/out";

    string armPart = rf.find("part").asString().c_str();
    string torsoPart = "torso";

    // prepare the option to open up the device driver to
    // access the robot
    Property optTorsoPart = newPartProperties(robot, local, torsoPart);
    Property optArmPart = newPartProperties(robot, local, armPart);

    // open the device driver
    partDrvTorso.open(optTorsoPart);
    partDrvTorso.view(posTorso);
    partDrvTorso.view(encTorso);
    partDrvArm.open(optArmPart);
    partDrvArm.view(posArm);
    partDrvArm.view(encArm);

    solverPort.open(localSolverName.c_str());
    solverCallback = new SolverProcessor(this);
    solverPort.useCallback(*solverCallback);

    //this can be done externally
    Network::connect(solverNameOut.c_str(), localSolverName.c_str());

    attachTerminal();

    //
    printf("configure() finished!\n");

    return true;
  }

  /**********************************************************/
  bool close()
  {
    if (server.isValid())
      server.close();
    if (partDrvTorso.isValid())
      partDrvTorso.close();
    if (partDrvArm.isValid())
      partDrvArm.close();

    printf("close() finished!\n");

    return true;
  }

  /**********************************************************/
  double getPeriod()
  {
    return .2;
  }

  bool updateModule()
  {
    /*bool *motionDone = new bool();
    double encsVal[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    posArm->checkMotionDone(motionDone);
    if (movingArm && *motionDone)
    {
      movingArm = false;
      encArm->getEncoders(encsVal);
      printf("Arm stopped at\t\t(\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f)\n", encsVal[0], encsVal[1], encsVal[2], encsVal[3],
             encsVal[4]);
    }
    posShoulder->checkMotionDone(motionDone);
    if (movingShoulder && *motionDone)
    {
      movingShoulder = false;
      encShoulder->getEncoders(encsVal);
      printf("Shoulder stopped at\t(\t%.2f\t%.2f\t%.2f)\n", encsVal[0], encsVal[1], encsVal[2]);
    }
    posTorso->checkMotionDone(motionDone);
    if (movingTorso && *motionDone)
    {
      movingTorso = false;
      encTorso->getEncoders(encsVal);
      printf("Torso stopped at\t(\t%.2f)\n", encsVal[0]);
    }*/
    return true;
  }

  bool respond(const Bottle & command, Bottle & reply)
  {
    if (command.get(0).asVocab() == VOCAB4('e','n','c','s'))
    {
      double encsVal[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      encTorso->getEncoders(encsVal);
      printf("Torso at\t(\t%.2f)\n", encsVal[0]);
      encArm->getEncoders(encsVal);
      printf("Arm at\t\t(\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f)\n", encsVal[0], encsVal[1], encsVal[2], encsVal[3], encsVal[4], encsVal[5], encsVal[6]);
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

  // register here the icubmod devices
  // along with the new yarp devices
  // for dealing with the robot

  ResourceFinder rf;
  rf.setVerbose(true);
  rf.setDefaultContext("vizzyCartesianControllerServer");
  rf.setDefaultConfigFile("testServer.ini");
  rf.configure(argc,argv);

  ServerModule server;
  return server.runModule(rf);
}

