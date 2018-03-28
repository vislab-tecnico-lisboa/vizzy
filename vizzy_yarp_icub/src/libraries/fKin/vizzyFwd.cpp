/*
 * vizzyFwd.cpp
 *
 *  Created on: Dec 27, 2011
 *      Author: vislab
 */

#include <stdio.h>
#include <sstream>

#include <vizzy/vizzyFwd.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

/************************************************************************/
vizzyEye::vizzyEye()
{
    allocate("right","waist");
}


/************************************************************************/
vizzyEye::vizzyEye(const string &_type)
{
    allocate(_type,"waist");
}

/************************************************************************/
vizzyEye::vizzyEye(const string &_type,const std::string &_root_link)
{
    allocate(_type,_root_link);
}

/************************************************************************/
vizzyEye::vizzyEye(const vizzyEye &eye)
{
    clone(eye);
}


/************************************************************************/
void vizzyEye::allocate(const string &_type, const std::string &_root_link)
{
    //iKinLimb::allocate(_type);
    type=_type;
    root_link=_root_link;
    configured=true;

    //virtual link
    //H0 (1.0 0.0 0.0 0.0   0.0 0.0 -1.0 0.0  0.0 1.0 0.0 0.0   0.0 0.0 0.0 1.0)     // given per rows (Very precise MATLAB Matrix)
    Matrix H0(4,4);
    H0.zero();
    if (_root_link=="base_link")
    {
	//1.0 0.0 0.0 0.189861 0.0 0.000004 -1.0 0.0 -0.0 1.0 -0.000004 0.535797 0.0 0.0 0.0 1.0)
        H0(0,0)=1.0;
        H0(1,2)=-1.0;
        H0(2,1)=1.0;
        H0(3,3)=1.0;
	H0(0,3)=0.189861;
	H0(2,3)=0.535797;
    }
    else if (_root_link=="waist")
    {
        H0(0,0)=1.0;
        H0(1,2)=-1.0;
        H0(2,1)=1.0;
        H0(3,3)=1.0;
    }

        setH0(H0);

      //pushLink(new iKinLink(  R,      D,      alpha,     theta,       minAngle,         maxAngle));
    yInfo(iKinLimb::getType());
    if (iKinLimb::getType()=="left" || iKinLimb::getType()=="left_v2" || iKinLimb::getType()=="left_v1")
    {
      /*
       Left eye
       Link    alpha   R       theta    D
       0       Pi/2    0       0        0       0/0     virtual link !!!NOT NEEDED!!!
       1       Pi/2    0       0        0       -20/20  M0 → M1
       2       Pi/2    0       0        -0.37   -53/53  M1 → M2
       3       Pi      0.13221 19*Pi/17 0       -18/37  M2 → M3
       4       -Pi/2   0       2*Pi/17  0.111  -38/38  M3 → M4
       //5       0       0.05    0        0       -38/38  M5 → End-effector
       5       0       0       Pi/2     Pi/2       -38/38  M5 → End-effector
       */
      pushLink(new iKinLink( 0, 	0,	M_PI/2.0,	0.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));//link1
      pushLink(new iKinLink( 0, 	-0.37,	M_PI/2.0,	0.0, -53.0*CTRL_DEG2RAD, 53.0*CTRL_DEG2RAD));//link2
      pushLink(new iKinLink( 0.13261, 	0, 	M_PI,		19.0*M_PI/17, -18.0*CTRL_DEG2RAD, 37.0*CTRL_DEG2RAD));//link3
      pushLink(new iKinLink( 0, 	0.1015,	M_PI/2.0,	-15.0*M_PI/17, -38.0*CTRL_DEG2RAD, 38.0*CTRL_DEG2RAD));//link4
      pushLink(new iKinLink( 0, 	0, 	M_PI/2.0,	-M_PI/2.0, -38.0*CTRL_DEG2RAD, 38.0*CTRL_DEG2RAD));//link5
    }
    else if (iKinLimb::getType()=="right" || iKinLimb::getType()=="right_v2" || iKinLimb::getType()=="right_v1")
    {
      /*
       Right eye
       Link    alpha   R       theta    D
       0       Pi/2    0       0        0       0/0     virtual link !!!NOT NEEDED!!!
       1       Pi/2    0       0        0       -20/20  M0 → M1
       2       Pi/2    0       0        -0.37   -53/53  M1 → M2
       3       Pi      0.13221 19*Pi/17 0       -18/37  M2 → M3
       4       -Pi/2   0       2*Pi/17  -0.111   -38/38  M3 → M5
       //5       0       0.05    0        0       -38/38  M5 → End-effector
       5       0       0       Pi/2     Pi/2       -38/38  M5 → End-effector
       */
      pushLink(new iKinLink( 0, 0, M_PI/2.0, 0.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));//link1
      pushLink(new iKinLink( 0, -0.37, M_PI/2.0, 0.0, -53.0*CTRL_DEG2RAD, 53.0*CTRL_DEG2RAD));//link2
      pushLink(new iKinLink( 0.13261, 0, M_PI, 19.0*M_PI/17, -18.0*CTRL_DEG2RAD, 37.0*CTRL_DEG2RAD));//link3
      pushLink(new iKinLink( 0, -0.1015, M_PI/2.0, -15.0*M_PI/17, -38.0*CTRL_DEG2RAD, 38.0*CTRL_DEG2RAD));//link4
      pushLink(new iKinLink( 0, 0, M_PI/2.0, -M_PI/2.0, -38.0*CTRL_DEG2RAD, 38.0*CTRL_DEG2RAD));//link5
    }
    Matrix HN(4,4);
    HN.zero();
    HN(0,0)=1.0;
    HN(1,1)=1.0;
    HN(2,2)=1.0;
    HN(3,3)=1.0;
    HN(2,3)=0.0275;
    setHN(HN);
    blockLink(0,0.0);//block torso
    //blockLink(1,0.0);//block neck yaw
    //blockLink(2,0.0);//block neck tilt
}


/************************************************************************/
bool vizzyEye::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<2)
        return false;

    IControlLimits &limTorso=*lim[0];
    IControlLimits &limHead =*lim[1];

    unsigned int iTorso;
    unsigned int iHead;
    double min, max;

    for (iTorso=0; iTorso<1; iTorso++)
    {
        if (!limTorso.getLimits(iTorso,&min,&max))
            return false;

        (*this)[/*2-*/iTorso].setMin(CTRL_DEG2RAD*min);
        (*this)[/*2-*/iTorso].setMax(CTRL_DEG2RAD*max);
    }

    for (iHead=0; iHead<getN()-iTorso; iHead++)
    {
        if (!limHead.getLimits(iHead,&min,&max))
            return false;

        (*this)[iTorso+iHead].setMin(CTRL_DEG2RAD*min);
        (*this)[iTorso+iHead].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}


/************************************************************************/
vizzyEyeNeckRef::vizzyEyeNeckRef()
{
    allocate("right");
}


/************************************************************************/
vizzyEyeNeckRef::vizzyEyeNeckRef(const string &_type)
{
    allocate(_type);
}


/************************************************************************/
vizzyEyeNeckRef::vizzyEyeNeckRef(const vizzyEyeNeckRef &eye)
{
    clone(eye);
}


/************************************************************************/
void vizzyEyeNeckRef::allocate(const string &_type)
{
    rmLink(0);
    rmLink(0);
    rmLink(0);

    delete linkList[0];
    delete linkList[1];
    delete linkList[2];

    linkList.erase(linkList.begin(),linkList.begin()+2);
}

void vizzyHeadCenter::allocate(const string &_type,const string &_root_link)
{
    iKinLimb::allocate(_type);

    // Vizzy head kinematics

    // accounting for the virtual link
    Matrix H0(4,4);
    H0.zero();
    if (_root_link=="base_link")
    {
    //1.0 0.0 0.0 0.189861 0.0 0.000004 -1.0 0.0 -0.0 1.0 -0.000004 0.535797 0.0 0.0 0.0 1.0)
        H0(0,0)=1.0;
        H0(1,2)=-1.0;
        H0(2,1)=1.0;
        H0(3,3)=1.0;
    H0(0,3)=0.189861;
    H0(2,3)=0.535797;
    }
    else if (_root_link=="waist")
    {
        H0(0,0)=1.0;
        H0(1,2)=-1.0;
        H0(2,1)=1.0;
        H0(3,3)=1.0;
    }
    setH0(H0);

/*#if 0
    pushLink(new iKinLink(    0.0,   0.0, M_PI/2.0,  0.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));
    pushLink(new iKinLink(    0.0, -0.37, M_PI/2.0,  0.0, -53.0*CTRL_DEG2RAD, 53.0*CTRL_DEG2RAD));
    pushLink(new iKinLink(0.13221,   0.0, M_PI,     M_PI, -18.0*CTRL_DEG2RAD, 37.0*CTRL_DEG2RAD));

    // align head-centered z-axis with the eyes z-axis
    Matrix HN(4,4);
    HN.zero();
    HN(2,0)=-1.0;
    HN(1,1)=1.0;
    HN(0,2)=1.0;
    HN(3,3)=1.0;
    setHN(HN);

    blockLink(0,0.0);   // block the torso link
#else*/
    pushLink(new iKinLink(    0.0,   0.0, M_PI/2.0,  0.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));
    pushLink(new iKinLink(    0.0, -0.37, M_PI/2.0,  0.0, -53.0*CTRL_DEG2RAD, 53.0*CTRL_DEG2RAD));
    pushLink(new iKinLink(0.13221,   0.0,     M_PI, M_PI, -18.0*CTRL_DEG2RAD, 37.0*CTRL_DEG2RAD));
    pushLink(new iKinLink(    0.0,   0.0,      0.0,  0.0, -38.0*CTRL_DEG2RAD, 18.0*CTRL_DEG2RAD));
    pushLink(new iKinLink(    0.0,   0.0,      0.0,  0.0, -38.0*CTRL_DEG2RAD, 38.0*CTRL_DEG2RAD));

    // align head-centered z-axis with the eyes z-axis
    Matrix HN(4,4);
    HN.zero();
    HN(2,0)=-1.0;
    HN(1,1)=1.0;
    HN(0,2)=1.0;
    HN(3,3)=1.0;
    setHN(HN);

    blockLink(0,0.0);   // block the torso link
    blockLink(3,0.0);   // block the eyes links
    blockLink(4,0.0);   // block the eyes links
//#endif
}

/************************************************************************/
vizzyInertialSensor::vizzyInertialSensor()
{
    allocate("","waist");
}


/************************************************************************/
vizzyInertialSensor::vizzyInertialSensor(const string &_type)
{
    allocate(_type,"waist");
}

/************************************************************************/
vizzyInertialSensor::vizzyInertialSensor(const string &_type, const std::string &_root_link)
{
    allocate(_type,_root_link);
}

/************************************************************************/
vizzyInertialSensor::vizzyInertialSensor(const vizzyInertialSensor &sensor)
{
    clone(sensor);
}


/************************************************************************/
void vizzyInertialSensor::allocate(const string &_type, const std::string &_root_link)
{
    iKinLimb::allocate(_type);

    //virtual link
    //H0 (1.0 0.0 0.0 0.0   0.0 0.0 -1.0 0.0  0.0 1.0 0.0 0.0   0.0 0.0 0.0 1.0)     // given per rows (Very precise MATLAB Matrix)
    Matrix H0(4,4);
    H0.zero();
    if (_root_link=="base_link")
    {
	//1.0 0.0 0.0 0.189861 0.0 0.000004 -1.0 0.0 -0.0 1.0 -0.000004 0.535797 0.0 0.0 0.0 1.0)
        H0(0,0)=1.0;
        H0(1,2)=-1.0;
        H0(2,1)=1.0;
        H0(3,3)=1.0;
	H0(0,3)=0.189861;
	H0(2,3)=0.535797;
    }
    else if (_root_link=="waist")
    {
        H0(0,0)=1.0;
        H0(1,2)=-1.0;
        H0(2,1)=1.0;
        H0(3,3)=1.0;
    }
    //pushLink(new iKinLink(  R,      D,      alpha,     theta,       minAngle,         maxAngle));

    /*
     Inertial sensor
     Link    alpha   R       theta      D       min/max
     0       Pi/2    0       0          0       0/0       virtual link
     1       Pi/2    0       0          0       -20/20    M0 → M1
     2       Pi/2    0       0          -0.37   -53/53    M1 → M2
     3       Pi      0.13221 -0.2611*Pi 0       -18/37    M2 → Inertial sensor
     4       Pi/2    0       0.7389*Pi  0       0/0       Orientation correction
     */
    pushLink(new iKinLink(       0,      0,   M_PI/2.0,           0.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));//link1
    pushLink(new iKinLink(       0,  -0.37,   M_PI/2.0,           0.0, -53.0*CTRL_DEG2RAD, 53.0*CTRL_DEG2RAD));//link2
    pushLink(new iKinLink( 0.13221,      0,       M_PI,  -0.2611*M_PI, -18.0*CTRL_DEG2RAD, 37.0*CTRL_DEG2RAD));//link3

    pushLink(new iKinLink(       0,      0,  M_PI/2.0,   0.7389*M_PI,                   0,                 0));//link4 (not a link)

    // block virtual links (in this case orientation link)
    blockLink(3,0.0);
}


/************************************************************************/
bool vizzyInertialSensor::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<2)
        return false;

    IControlLimits &limTorso=*lim[0];
    IControlLimits &limHead =*lim[1];

    unsigned int iTorso;
    unsigned int iHead;
    double min, max;

    for (iTorso=0; iTorso<1; iTorso++)
    {
        if (!limTorso.getLimits(iTorso,&min,&max))
            return false;

        (*this)[/*2-*/iTorso].setMin(CTRL_DEG2RAD*min);
        (*this)[/*2-*/iTorso].setMax(CTRL_DEG2RAD*max);
    }

    // only the neck
    for (iHead=0; iHead<2; iHead++)
    {
        if (!limHead.getLimits(iHead,&min,&max))
            return false;

        (*this)[iTorso+iHead].setMin(CTRL_DEG2RAD*min);
        (*this)[iTorso+iHead].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}

