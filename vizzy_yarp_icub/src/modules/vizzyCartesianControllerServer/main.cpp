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

#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

/**
 * This class launches the server.
 */
class ServerModule: public RFModule
{
protected:
    PolyDriver partDrvTorso,partDrvArm;
    PolyDriver server;

private:
    Property newPartProperties(string robot, string srvName, string part){
            Property optPart;
            optPart.put("device","remote_controlboard");
            optPart.put("remote",("/"+robot+"/"+part).c_str());
            optPart.put("local",("/"+robot+"/"+srvName+"/"+part).c_str());
            optPart.put("robot",robot.c_str());
            optPart.put("part",part.c_str());
            return optPart;
      }

public:
    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {   
        // grab parameters from the congiguration file
        string robot=rf.find("robot").asString().c_str();
        string local=rf.find("local").asString().c_str();
        string pathToKin=rf.findFile("kinematics_file").c_str();
        cout<<"Everything seems fine!"<<endl;

        string armPart=rf.find("part").asString().c_str();
        string torsoPart="torso";
        // prepare the option to open up the device driver to
        // access the robot
        Property optTorsoPart = newPartProperties(robot,local,torsoPart);
        Property optArmPart = newPartProperties(robot,local,armPart);

        // open the device driver
        partDrvTorso.open(optTorsoPart);
        partDrvArm.open(optArmPart);
        /*if (!partDrvTorso.open(optTorsoPart) && !partDrvShoulder.open(optShoulderPart) && !partDrvArm.open(optArmPart))
        {
            cout<<"Error: Device driver not available!"<<endl;
            close();
            return false;
        }*/

        // now go on with the server driver
        PolyDriverList list;
        list.push(&partDrvTorso,torsoPart.c_str());
        list.push(&partDrvArm,armPart.c_str());

        // take the parameters and fill the kinematic description
        Property optServer("(device cartesiancontrollerserver)");
        optServer.fromConfigFile(rf.findFile("from").c_str(),false);
        Bottle &kinInfo=optServer.findGroup("GENERAL").addList();
        kinInfo.addString("CustomKinFile");
        kinInfo.addString(pathToKin.c_str());

        if (!server.open(optServer))
        {
            cout<<"Error: Unable to open the Cartesian Controller Server!"<<endl;
            close();    
            return false;
        }

        // attach the device driver to the server
        IMultipleWrapper *wrapper;
        server.view(wrapper);
        if (!wrapper->attachAll(list))
        {
            cout<<"Error: Unable to attach device drivers!"<<endl;
            close();    
            return false;
        }

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


        return true;
    }

    /**********************************************************/
    double getPeriod()    { return 1.0;  }
    bool   updateModule() { return true; }
};


/**********************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"Error: yarp server does not seem available"<<endl;
        return -1;
    }

    // register here the icubmod devices
    // along with the new yarp devices
    // for dealing with the robot

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("vizzyCartesianControllerServer");
    rf.setDefaultConfigFile("server_left_arm.ini");
    rf.configure(argc,argv);

    ServerModule server;
    return server.runModule(rf);
}



