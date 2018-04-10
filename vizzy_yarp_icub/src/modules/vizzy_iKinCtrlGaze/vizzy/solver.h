/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
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

#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/pids.h>

#include <vizzy/gazeNlp.h>
#include <vizzy/utils.h>
#include <vizzy/localizer.h>
#include <vizzy/controller.h>

#define EYEPINVREFGEN_GAIN                  12.5    // [-]
#define SACCADES_INHIBITION_PERIOD          0.2     // [s]
#define SACCADES_VEL                        1000.0  // [deg/s]
#define SACCADES_ACTIVATIONANGLE            10.0    // [deg]
#define GYRO_BIAS_STABILITY                 5.0     // [deg/s]
#define NECKSOLVER_ACTIVATIONDELAY          0.25    // [s]
#define NECKSOLVER_ACTIVATIONANGLE_JOINTS   0.1     // [deg]
#define NECKSOLVER_ACTIVATIONANGLE          2.5     // [deg]
#define NECKSOLVER_RESTORINGANGLE           5.0     // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// The thread launched by the application which computes
// the eyes target position relying on the pseudoinverse
// method.
class EyePinvRefGen : public RateThread
{
protected:
    vizzyHeadCenter       *neck;
    vizzyEye              *eyeL,      *eyeR;
    iKinChain            *chainNeck, *chainEyeL, *chainEyeR;
    vizzyInertialSensor    inertialSensor;
    PolyDriver           *drvTorso,  *drvHead;
    exchangeData         *commData;
    string robotName;
    Controller           *ctrl;
    xdPort               *port_xd;
    Integrator           *I;    

    BufferedPort<Vector> port_inertial;
    Semaphore mutex;

    string localName;
    string camerasFile;
    double eyeTiltMin;
    double eyeTiltMax;
    bool saccadesOn;
    bool Robotable;
    bool headV2;
    unsigned int period;
    bool saccadeUnderWayOld;
    bool genOn;
    int nJointsTorso;
    int nJointsHead;
    int    saccadesRxTargets;
    double saccadesClock;
    double saccadesInhibitionPeriod;
    double eyesHalfBaseline;
    double Ts;
    ResourceFinder rf_camera;
    
    Matrix lim;
    Vector fbTorso;
    Vector fbHead;
    Vector qd,fp;
    Matrix eyesJ;
    Vector gyro;
    Vector counterRotGain;

    Vector getEyesCounterVelocity(const Matrix &eyesJ, const Vector &fp);

public:
    EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
                  const string &_robotName, Controller *_ctrl, const string &_localName,
                  ResourceFinder &_camerasFile, const double _eyeTiltMin, const double _eyeTiltMax,
                  const bool _saccadesOn, const Vector &_counterRotGain, const bool _headV2,
                  const string &_root_link,const unsigned int _period);

    void   set_xdport(xdPort *_port_xd)                        { port_xd=_port_xd;                   }
    void   enable()                                            { genOn=true;                         }
    void   disable()                                           { genOn=false;                        }    
    Vector getCounterRotGain() const                           { return counterRotGain;              }
    void   setSaccades(const bool sw)                          { saccadesOn=sw;                      }
    bool   isSaccadesOn() const                                { return saccadesOn;                  }
    void   setSaccadesInhibitionPeriod(const double inhPeriod) { saccadesInhibitionPeriod=inhPeriod; }
    double getSaccadesInhibitionPeriod() const                 { return saccadesInhibitionPeriod;    }
    void   setCounterRotGain(const Vector &gain);
    bool   getGyro(Vector &data);    
    bool   threadInit();
    void   afterStart(bool s);
    void   run();
    void   threadRelease();
    void   suspend();
    void   resume();
    void   stopControl();
};


// The thread launched by the application which is
// in charge of inverting the head kinematic relying
// on IPOPT computation.
class Solver : public RateThread
{
protected:    
    vizzyHeadCenter     *neck;
    vizzyEye            *eyeL,      *eyeR;
    iKinChain          *chainNeck, *chainEyeL, *chainEyeR;
    vizzyInertialSensor  inertialSensor;
    GazeIpOptMin       *invNeck;
    PolyDriver         *drvTorso,  *drvHead;
    exchangeData       *commData;
    EyePinvRefGen      *eyesRefGen;
    Localizer          *loc;
    Controller         *ctrl;
    xdPort             *port_xd;
    Semaphore           mutex;

    string localName;
    string camerasFile;
    double eyeTiltMin;
    double eyeTiltMax;
    bool headV2;
    unsigned int period;
    bool Robotable;
    bool bindSolveRequest;
    int nJointsTorso;
    int nJointsHead;
    double Ts;
    ResourceFinder rf_camera;

    Vector fbTorso;
    Vector fbHead;
    Vector neckPos;
    Vector gazePos;
    Vector gDefaultDir;
    Vector fbTorsoOld;
    Vector fbHeadOld;

    double neckPitchMin;
    double neckPitchMax;
    double neckRollMin;
    double neckRollMax;
    double neckYawMin;
    double neckYawMax;

    void   updateAngles();
    Vector getGravityDirection(const Vector &gyro);

public:
    Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
           EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
           const string &_localName, ResourceFinder &_camerasFile, const double _eyeTiltMin,
           const double _eyeTiltMax, const bool _headV2, const string &_root_link,const unsigned int _period);

    // Returns a measure of neck angle required to reach the target
    Vector neckTargetRotAngles(const Vector &xd);    
    void   bindNeckPitch(const double min_deg, const double max_deg);
    void   bindNeckRoll(const double min_deg, const double max_deg);
    void   bindNeckYaw(const double min_deg, const double max_deg);
    void   getCurNeckPitchRange(double &min_deg, double &max_deg);
    void   getCurNeckRollRange(double &min_deg, double &max_deg);
    void   getCurNeckYawRange(double &min_deg, double &max_deg);
    void   clearNeckPitch();
    void   clearNeckRoll();
    void   clearNeckYaw();    
    bool   threadInit();
    void   afterStart(bool s);
    void   run();
    void   threadRelease();
    void   suspend();
    void   resume();
};


#endif


