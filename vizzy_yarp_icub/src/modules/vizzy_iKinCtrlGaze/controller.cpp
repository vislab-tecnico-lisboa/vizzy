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
 * http://www.robotcub.org/vizzy/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <algorithm>

#include <yarp/os/Time.h>
#include <vizzy/solver.h>
#include <vizzy/controller.h>


/************************************************************************/
Controller::Controller(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
                       const string &_robotName, const string &_localName, ResourceFinder &_camerasFile,
                       const double _neckTime, const double _eyesTime, const double _eyeTiltMin,
                       const double _eyeTiltMax, const double _minAbsVel, const bool _headV2,
                       const string &_root_link, const unsigned int _period) :
                       RateThread(_period),       drvTorso(_drvTorso),     drvHead(_drvHead),
                       commData(_commData),       robotName(_robotName),   localName(_localName),
                        neckTime(_neckTime),     eyesTime(_eyesTime),
                       eyeTiltMin(_eyeTiltMin),   eyeTiltMax(_eyeTiltMax), minAbsVel(_minAbsVel),
                       headV2(_headV2),           period(_period),         Ts(_period/1000.0),
                       printAccTime(0.0)
{
    this->rf_camera=_camerasFile;
    Robotable=(drvHead!=NULL);
     fprintf(stdout,"Robotable = %d \n",Robotable);
    // Instantiate objects
    neck=new vizzyHeadCenter(headV2?"right_v2":"right",_root_link);
    eyeL=new vizzyEye(headV2?"left_v2":"left",_root_link);
    eyeR=new vizzyEye(headV2?"right_v2":"right",_root_link);

    // release links
    neck->releaseLink(0); eyeL->releaseLink(0); eyeR->releaseLink(0);
    neck->releaseLink(1); eyeL->releaseLink(1); eyeR->releaseLink(1);
    /*neck->releaseLink(2);*/ eyeL->releaseLink(2); eyeR->releaseLink(2);

    // Get the chain objects
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();
    chainEyeR=eyeR->asChain();

    // add aligning matrices read from configuration file
    getAlignHN(rf_camera,"ALIGN_KIN_LEFT",eyeL->asChain(),true);
    getAlignHN(rf_camera,"ALIGN_KIN_RIGHT",eyeR->asChain(),true);

    if (Robotable)
    {
        // read number of joints
        if (drvTorso!=NULL)
        {
            IEncoders *encTorso; drvTorso->view(encTorso);
            encTorso->getAxes(&nJointsTorso);
        }
        else
            nJointsTorso=1;
        
        IEncoders *encHead; drvHead->view(encHead);
        encHead->getAxes(&nJointsHead);

	drvHead->view(modHead);
        drvHead->view(posHead);
        drvHead->view(velHead);

        // joints bounds alignment
        lim=alignJointsBounds(chainNeck,drvTorso,drvHead,eyeTiltMin,eyeTiltMax);
        copyJointsBounds(chainNeck,chainEyeL);
        copyJointsBounds(chainEyeL,chainEyeR);

        // read starting position
        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);

        // exclude acceleration constraints by fixing
        // thresholds at high values
        Vector a_robHead(nJointsHead,1e9);
        velHead->setRefAccelerations(a_robHead.data());
    }
    else
    {
        nJointsTorso=1;
        nJointsHead =5;
        
        // create bounds matrix for integrators
        lim.resize(nJointsHead,2);
        for (int i=0; i<nJointsHead-1; i++)
        {
            lim(i,0)=(*chainNeck)[nJointsTorso+i].getMin();
            lim(i,1)=(*chainNeck)[nJointsTorso+i].getMax();
        }

        // vergence
        lim(nJointsHead-1,1)=lim(nJointsHead-2,1);

        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);
    }

    // find minimum allowed vergence
    findMinimumAllowedVergence();

    // reinforce vergence min bound
    lim(nJointsHead-1,0)=commData->get_minAllowedVergence();

    if (Robotable)
        getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
    else
        fbHead[4]=commData->get_minAllowedVergence();

    fbNeck.resize(2); fbEyes.resize(3);
    qdNeck.resize(2); qdEyes.resize(3);
    vNeck.resize(2);  vEyes.resize(3);

    // Set the task execution time
    setTeyes(eyesTime);
    setTneck(neckTime);

    mjCtrlNeck=new minJerkVelCtrlForIdealPlant(Ts,fbNeck.length());
    mjCtrlEyes=new minJerkVelCtrlForIdealPlant(Ts,fbEyes.length());
    Int=new Integrator(Ts,fbHead,lim);
    
    v.resize(nJointsHead,0.0);
    vdegOld=v;

    qd=fbHead;
    qddeg=CTRL_RAD2DEG*qd;
    vdeg =CTRL_RAD2DEG*v;

    port_xd=NULL;
    saccadeStartTime=0.0;
    tiltDone=panDone=verDone=false;
    unplugCtrlEyes=false;
}


/************************************************************************/
void Controller::findMinimumAllowedVergence()
{
    iKinChain cl(*chainEyeL), cr(*chainEyeR);
    Vector zeros(cl.getDOF(),0.0);
    cl.setAng(zeros); cr.setAng(zeros);

    double minVer=0.5*CTRL_DEG2RAD;
    double maxVer=lim(nJointsHead-1,1);
    for (double ver=0.0; ver<maxVer; ver+=0.5*CTRL_DEG2RAD)
    {
        cl(cl.getDOF()-1).setAng(ver/2.0);
        cr(cr.getDOF()-1).setAng(-ver/2.0);

        Vector fp(4);
        fp[3]=1.0;  // impose homogeneous coordinates
        if (computeFixationPointOnly(cl,cr,fp))
        {
            // if the component along eye's z-axis is positive
            // then this means that the fixation point is ok,
            // being in front of the robot
            Vector fpe=SE3inv(cl.getH())*fp;
            if (fpe[2]>0.0)
            {
                minVer=ver;
                break;
            }
        }
    }

    fprintf(stdout,"### computed minimum allowed vergence = %g [deg]\n",minVer*CTRL_RAD2DEG);
    commData->get_minAllowedVergence()=minVer;
}

/************************************************************************/
/*void Controller::notifyEvent(const string &event, const double checkPoint)
{
    if (port_event.getOutputCount()>0)
    {
        Bottle &ev=port_event.prepare();
        ev.clear();

        ev.addString(event.c_str());
        ev.addDouble(q_stamp);
        if (checkPoint>=0.0)
            ev.addDouble(checkPoint);

        txInfo_event.update(q_stamp);
        port_event.setEnvelope(txInfo_event);
        port_event.writeStrict();
    }
}*/

/************************************************************************/
void Controller::stopLimbsVel()
{
    if (Robotable)
    {
        // this timeout prevents the stop() from
        // being overwritten by the last velocityMove()
        // which travels on a different connection.
        Time::delay(2*Ts);

        mutexCtrl.wait();
        velHead->stop();
        mutexCtrl.post();
    }
}


/************************************************************************/
void Controller::printIter(Vector &xd, Vector &fp, Vector &qd, Vector &q,
                           Vector &v, double printTime)
{
    if ((printAccTime+=Ts)>=printTime)
    {
        printAccTime=0.0;

        fprintf(stdout,"\n");
        fprintf(stdout,"norm(e)           = %g\n",norm(xd-fp));
        fprintf(stdout,"Target fix. point = %s\n",xd.toString().c_str());
        fprintf(stdout,"Actual fix. point = %s\n",fp.toString().c_str());
        fprintf(stdout,"Target Joints     = %s\n",qd.toString().c_str());
        fprintf(stdout,"Actual Joints     = %s\n",q.toString().c_str());
        fprintf(stdout,"Velocity          = %s\n",v.toString().c_str());
        fprintf(stdout,"\n");
    }
}


/************************************************************************/
bool Controller::threadInit()
{
    port_x.open((localName+"/x:o").c_str());
    port_q.open((localName+"/q:o").c_str());
    port_event.open((localName+"/events:o").c_str());

    fprintf(stdout,"Starting Controller at %d ms\n",period);

    return true;
}


/************************************************************************/
void Controller::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Controller started successfully\n");
    else
        fprintf(stdout,"Controller did not start\n");
}


/************************************************************************/
void Controller::doSaccade(Vector &ang, Vector &vel)
{
    mutexCtrl.wait();

    modHead->setControlMode(2,VOCAB_CM_POSITION);
    modHead->setControlMode(3,VOCAB_CM_POSITION);
    modHead->setControlMode(4,VOCAB_CM_POSITION);
    posHead->setRefSpeed(2,CTRL_RAD2DEG*vel[0]);
    posHead->setRefSpeed(3,CTRL_RAD2DEG*vel[1]);
    posHead->setRefSpeed(4,CTRL_RAD2DEG*vel[2]);

    // enforce joints bounds
    ang[0]=std::min(std::max(lim(3,0),ang[0]),lim(3,1));
    ang[1]=std::min(std::max(lim(4,0),ang[1]),lim(4,1));
    ang[2]=std::min(std::max(lim(5,0),ang[2]),lim(5,1));

    posHead->positionMove(2,CTRL_RAD2DEG*ang[0]);
    posHead->positionMove(3,CTRL_RAD2DEG*ang[1]);
    posHead->positionMove(4,CTRL_RAD2DEG*ang[2]);

    saccadeStartTime=Time::now();
    tiltDone=panDone=verDone=false;
    commData->get_isSaccadeUnderway()=true;
    unplugCtrlEyes=true;

    mutexCtrl.post();    
}


/************************************************************************/
void Controller::resetCtrlEyes()
{
    mutexCtrl.wait();
    mjCtrlEyes->reset(zeros(3));
    unplugCtrlEyes=false;
    mutexCtrl.post();
}


/************************************************************************/
void Controller::run()
{
    // verify if any saccade is still underway
    mutexCtrl.wait();
    modHead->setControlMode(0,VOCAB_CM_VELOCITY);
    modHead->setControlMode(1,VOCAB_CM_VELOCITY);
    modHead->setControlMode(2,VOCAB_CM_VELOCITY);
    modHead->setControlMode(3,VOCAB_CM_MIXED);
    modHead->setControlMode(4,VOCAB_CM_MIXED);
    if (commData->get_isSaccadeUnderway() && (Time::now()-saccadeStartTime>=Ts))
    {
        if (!tiltDone)
            posHead->checkMotionDone(2,&tiltDone);

        if (!panDone)
            posHead->checkMotionDone(3,&panDone);

        if (!verDone)
            posHead->checkMotionDone(4,&verDone);

        commData->get_isSaccadeUnderway()=!(tiltDone&&panDone&&verDone);
    }
    mutexCtrl.post();
    
    // get data
    double x_stamp;
    Vector xd=commData->get_xd();
    Vector x=commData->get_x(x_stamp);
    Vector new_qd=commData->get_qd();

    double errNeck=norm(new_qd.subVector(0,1)-fbHead.subVector(0,1));
    double errEyes=norm(new_qd.subVector(2,new_qd.length()-1)-fbHead.subVector(2,fbHead.length()-1));
    bool swOffCond=!commData->get_isSaccadeUnderway() &&
                   (errNeck<GAZECTRL_MOTIONDONE_NECK_QTHRES*CTRL_DEG2RAD) &&
                   (errEyes<GAZECTRL_MOTIONDONE_EYES_QTHRES*CTRL_DEG2RAD);

    // verify control switching conditions
    if (commData->get_isCtrlActive())
    {
        // switch-off condition
        if (swOffCond)
        {
            stopLimbsVel();
            commData->get_isCtrlActive()=false;
            port_xd->get_new()=false;
        }
    }
    else if (!swOffCond)
    {
        // switch-on condition
        commData->get_isCtrlActive()=(new_qd[0]!=qd[0]) || (new_qd[1]!=qd[1]) ||
                                     (commData->get_canCtrlBeDisabled() ?
                                      port_xd->get_new() : (norm(port_xd->get_xd()-x)>GAZECTRL_MOTIONSTART_XTHRES));

        // reset controllers
        if (commData->get_isCtrlActive())
        {
        	Vector zeros(3,0.0);
        	Vector zeros2(2,0.0);
            mjCtrlNeck->reset(zeros2);
            mjCtrlEyes->reset(zeros);
        }
    }

    // Introduce the feedback within the control computation
    double q_stamp=Time::now();
    if (Robotable)
    {
        if (!getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData,&q_stamp))
        {
            fprintf(stdout,"\nCommunication timeout detected!\n\n");
            suspend();

            return;
        }

        Int->reset(fbHead);
    }

    qd=new_qd;
    for (int i=0; i<3; i++)
    {
    	if(i<2) qdNeck[i]=qd[i];
        qdEyes[i]=qd[2+i];

        if(i<2) fbNeck[i]=fbHead[i];
        fbEyes[i]=fbHead[2+i];
    }

    vNeck=0.0; vEyes=0.0;
    if (commData->get_isCtrlActive())
    {
        // control loop
        vNeck=mjCtrlNeck->computeCmd(neckTime,qdNeck-fbNeck);

        if (unplugCtrlEyes)
        {
            if (Time::now()-saccadeStartTime>=Ts)
                vEyes=commData->get_counterv();
        }
        else
            vEyes=mjCtrlEyes->computeCmd(eyesTime,qdEyes-fbEyes)+commData->get_counterv();
    }

    for (int i=0; i<3; i++)
    {
    	if(i<2) v[i]  =vNeck[i];
        v[2+i]=vEyes[i];
    }

    // apply bang-bang just in case to compensate
    // for unachievable low velocities
    if (Robotable)
    {
        for (size_t i=0; i<v.length(); i++)
            if ((v[i]>-minAbsVel) && (v[i]<minAbsVel) && (v[i]!=0.0))
                v[i]=yarp::math::sign(qd[i]-fbHead[i])*minAbsVel;
    }

    // convert to degrees
    mutexData.wait();
    qddeg=CTRL_RAD2DEG*qd;
    qdeg =CTRL_RAD2DEG*fbHead;
    vdeg =CTRL_RAD2DEG*v;
    mutexData.post();

    // send velocities to the robot
    if (Robotable && !(vdeg==vdegOld))
    {
        mutexCtrl.wait();
        velHead->velocityMove(vdeg.data());
        mutexCtrl.post();
        vdegOld=vdeg;
    }

    // print info
    printIter(xd,x,qddeg,qdeg,vdeg,1.0);

    // send x,q through YARP ports
    Vector q(nJointsTorso+nJointsHead);
    int j;
    for (j=0; j<nJointsTorso; j++)
        q[j]=CTRL_RAD2DEG*fbTorso[j];
    for (; (size_t)j<q.length(); j++)
        q[j]=qdeg[j-nJointsTorso];

    txInfo_x.update(x_stamp);
    if (port_x.getOutputCount()>0)
    {        
        port_x.setEnvelope(txInfo_x);
        port_x.write(x);
    }

    txInfo_q.update(q_stamp);
    if (port_q.getOutputCount()>0)
    {        
        port_q.setEnvelope(txInfo_q);
        port_q.write(q);
    }

    // update pose information
    mutexChain.wait();

    for (int i=0; i<nJointsTorso; i++)
    {
        chainNeck->setAng(i,fbTorso[i]);
        chainEyeL->setAng(i,fbTorso[i]);
        chainEyeR->setAng(i,fbTorso[i]);
    }
    for (int i=0; i<3; i++)
    {
    	chainNeck->setAng(nJointsTorso+i,fbHead[i]);
        chainEyeL->setAng(nJointsTorso+i,fbHead[i]);
        chainEyeR->setAng(nJointsTorso+i,fbHead[i]);
    }
    chainEyeL->setAng(nJointsTorso+2,fbHead[2]);               chainEyeR->setAng(nJointsTorso+2,fbHead[2]);
    //chainEyeL->setAng(nJointsTorso+3,fbHead[3]+fbHead[4]/2.0); chainEyeR->setAng(nJointsTorso+3,fbHead[3]-fbHead[4]/2.0);
    chainEyeL->setAng(nJointsTorso+3,(fbHead[3]+fbHead[4])/2.0); chainEyeR->setAng(nJointsTorso+3,(fbHead[3]-fbHead[4])/2.0);

    txInfo_pose.update(q_stamp);

    mutexChain.post();

    // update joints angles
    fbHead=Int->integrate(v);
    commData->set_q(fbHead);
    commData->set_torso(fbTorso);
    commData->set_v(v);
}


/************************************************************************/
void Controller::threadRelease()
{
    stopLimbsVel();

    port_x.interrupt();
    port_q.interrupt();
    port_event.interrupt();

    port_x.close();
    port_q.close();

    delete neck;
    delete eyeL;
    delete eyeR;
    delete mjCtrlNeck;
    delete mjCtrlEyes;
    delete Int;
}


/************************************************************************/
void Controller::suspend()
{
    stopLimbsVel();

    fprintf(stdout,"\nController has been suspended!\n\n");

    RateThread::suspend();
}


/************************************************************************/
void Controller::resume()
{
    if (Robotable)
    {
        getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
    
        for (int i=0; i<3; i++)
        {
            fbNeck[i]=fbHead[i];
            fbEyes[i]=fbHead[2+i];
        }
    }

    fprintf(stdout,"\nController has been resumed!\n\n");

    RateThread::resume();
}


/************************************************************************/
double Controller::getTneck() const
{
    return neckTime;
}


/************************************************************************/
double Controller::getTeyes() const
{
    return eyesTime;
}


/************************************************************************/
void Controller::setTneck(const double execTime)
{
    double lowerThresNeck=eyesTime+0.2;
    if (execTime<lowerThresNeck)
    {        
        fprintf(stdout,"Warning: neck execution time is under the lower bound!\n");
        fprintf(stdout,"A new neck execution time of %g s is chosen\n",lowerThresNeck);

        neckTime=lowerThresNeck;
    }
    else
        neckTime=execTime;
}


/************************************************************************/
void Controller::setTeyes(const double execTime)
{
    double lowerThresEyes=10.0*Ts;
    if (execTime<lowerThresEyes)
    {        
        fprintf(stdout,"Warning: eyes execution time is under the lower bound!\n");
        fprintf(stdout,"A new eyes execution time of %g s is chosen\n",lowerThresEyes);

        eyesTime=lowerThresEyes;
    }
    else
        eyesTime=execTime;
}


/************************************************************************/
bool Controller::isMotionDone() const
{
    return !commData->get_isCtrlActive();
}


/************************************************************************/
void Controller::setTrackingMode(const bool f)
{
    commData->get_canCtrlBeDisabled()=!f;

    if ((port_xd!=NULL) && f)
        port_xd->set_xd(commData->get_x());
}


/************************************************************************/
bool Controller::getTrackingMode() const 
{
    return !commData->get_canCtrlBeDisabled();
}


/************************************************************************/
bool Controller::getDesired(Vector &des)
{
    mutexData.wait();
    des=qddeg;
    mutexData.post();
    return true;
}


/************************************************************************/
bool Controller::getVelocity(Vector &vel)
{
    mutexData.wait();
    vel=vdeg;
    mutexData.post();
    return true;
}


/************************************************************************/
bool Controller::getPose(const string &poseSel, Vector &x, Stamp &stamp)
{
    if (poseSel=="left")
    {
        mutexChain.wait();
        x=chainEyeL->EndEffPose();
        stamp=txInfo_pose;
        mutexChain.post();
        return true;
    }
    else if (poseSel=="right")
    {
        mutexChain.wait();
        x=chainEyeR->EndEffPose();
        stamp=txInfo_pose;
        mutexChain.post();
        return true;
    }
    else if (poseSel=="head")
    {
        mutexChain.wait();
        x=chainNeck->EndEffPose();
        stamp=txInfo_pose;
        mutexChain.post();
        return true;
    }
    else
        return false;
}


