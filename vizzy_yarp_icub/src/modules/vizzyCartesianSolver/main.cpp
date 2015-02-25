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
#include <iCub/iKin/iKinSlv.h>

#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp::os;
using namespace iCub::iKin;

/**
 * This class inherits from the CartesianSolver super-class
 * implementing the solver
 */
class VizzyRobotCartesianSolver : public CartesianSolver
{

private:
    Property newPartProperties(string robot, string slvName, string part){
            Property optPart;
            optPart.put("device","remote_controlboard");
            optPart.put("remote",("/"+robot+"/"+part).c_str());
            optPart.put("local",("/"+robot+"/"+slvName+"/"+part).c_str());
            optPart.put("robot",robot.c_str());
            optPart.put("part",part.c_str());
            return optPart;
      }

protected:
    /**
     * This particular method serves to describe all the device
     * drivers used by the solver to access the robot, along with
     * the kinematic structure of the links.
     * 
     * @param options The parameters required by the super-class to 
     *                get configured.
     * @return A pointer to the descriptor or NULL if something wrong happens.
     */
    PartDescriptor *getPartDesc(Searchable &options)
    {
        if (!options.check("customKinFile"))
        {
            cout<<"Error: \"customKinFile\" option is missing!"<<endl;
            return NULL;
        }

        string robot=options.find("robot").asString().c_str();
        string armPart=options.find("part").asString().c_str();
        string torsoPart="torso";

        // here we declare everything is required to open up
        // the device driver to access the robot
        Property optTorsoPart = newPartProperties(robot,slvName,torsoPart);
        Property optArmPart = newPartProperties(robot,slvName,armPart);

        // we grab info on the robot's kinematics
        Property linksOptions;
        linksOptions.fromConfigFile(options.find("customKinFile").asString().c_str());
        iKinLimb *limb=new iKinLimb(linksOptions);
        if (!limb->isValid())
        {
            cout<<"Error: invalid links parameters!"<<endl;
            delete limb;
            return NULL;
        }

        // we fill in the descriptor fields
        PartDescriptor *p=new PartDescriptor;
        p->lmb=limb;                // a pointer to the iKinLimb
        p->chn=limb->asChain();     // the associated iKinChain object
        p->cns=NULL;                // any further (linear) constraints on the joints other than the bounds? This requires some more effort
        //p->prp.push_back(optArmPart);  // attach the options to open the device driver of the fake part
        //p->rvs.push_back(false);    // it may happen that the motor commands to be sent are in reversed order wrt the order of kinematics links (e.g. the iCub torso); if so put here "true"
        //p->num=1;                   // only one device driver for the whole limb (see below)

        // whenever a limb is actuated resorting to more than one device
        // (e.g. for iCub: torso+arm), the following applies:
        // 
        p->prp.push_back(optTorsoPart);
        p->prp.push_back(optArmPart);
        p->rvs.push_back(false);
        p->rvs.push_back(false);
        p->num=2;

        return p;
    }

public:
    /**********************************************************/
    VizzyRobotCartesianSolver(const string &_slvName="solver") : CartesianSolver(_slvName) { }
};

class SolverModule: public RFModule
{
protected:
    CartesianSolver *solver;

public:
    /**********************************************************/
    SolverModule() : solver(NULL) { }

    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {                
        if (!rf.check("name"))
        {
            cout<<"Error: \"name\" option is missing!"<<endl;
            return false;
        }

        string solverName=rf.find("name").asString().c_str();
        string pathToKin=rf.findFile("kinematics_file").c_str();

        Property config;
        config.fromConfigFile(rf.findFile("from").c_str());
        config.put("customKinFile",pathToKin.c_str());

        solver=new VizzyRobotCartesianSolver(solverName);
        if (!solver->open(config))
        {    
            delete solver;
            return false;
        }

        return true;
    }

    /**********************************************************/
    bool close()
    {
        if (solver!=NULL)
            delete solver;

        return true;
    }

    /**********************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /**********************************************************/
    bool updateModule()
    {
        if (solver->isClosed() || solver->getTimeoutFlag())
            return false;
        else
            return true;
    }
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

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("solver.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    SolverModule solver;
    return solver.runModule(rf);
}



