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
#include <yarp/math/SVD.h>
#include <vizzy/solver.h>


/************************************************************************/
EyePinvRefGen::EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead,
                             exchangeData *_commData, const string &_robotName,
                             Controller *_ctrl, const string &_localName,
                             ResourceFinder &_camerasFile, const double _eyeTiltMin,
                             const double _eyeTiltMax, const bool _saccadesOn,
                             const Vector &_counterRotGain, const bool _headV2,
                             const string &_root_link,const unsigned int _period) :
                             RateThread(_period),     drvTorso(_drvTorso),       drvHead(_drvHead),
                             commData(_commData),     robotName(_robotName),     ctrl(_ctrl),
                             localName(_localName),   eyeTiltMin(_eyeTiltMin),
                             eyeTiltMax(_eyeTiltMax), saccadesOn(_saccadesOn),   headV2(_headV2),
                             period(_period),         Ts(_period/1000.0)
{
    this->rf_camera=_camerasFile;
    Robotable=(drvHead!=NULL);
    counterRotGain=_counterRotGain;

    // Instantiate objects
    neck=new vizzyHeadCenter(headV2?"right_v2":"right",_root_link);
    eyeL=new vizzyEye(headV2?"left_v2":"left",_root_link);
    eyeR=new vizzyEye(headV2?"right_v2":"right",_root_link);

    // remove constraints on the links: logging purpose
    inertialSensor.setAllConstraints(false);

    // block neck dofs
    eyeL->blockLink(1,0.0); eyeR->blockLink(1,0.0);
    eyeL->blockLink(2,0.0); eyeR->blockLink(2,0.0);


    // Get the chain objects
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();
    chainEyeR=eyeR->asChain();

    // add aligning matrices read from configuration file
    getAlignHN(rf_camera,"ALIGN_KIN_LEFT",eyeL->asChain(),true);
    getAlignHN(rf_camera,"ALIGN_KIN_RIGHT",eyeR->asChain(),true);

    // get the lenght of the half of the eyes baseline
    eyesHalfBaseline=0.5*norm(eyeL->EndEffPose().subVector(0,2)-eyeR->EndEffPose().subVector(0,2));
    
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

        // joints bounds alignment
        lim=alignJointsBounds(chainNeck,drvTorso,drvHead,eyeTiltMin,eyeTiltMax);
        copyJointsBounds(chainNeck,chainEyeL);
        copyJointsBounds(chainEyeL,chainEyeR);

        // reinforce vergence min bound
        lim(nJointsHead-1,0)=commData->get_minAllowedVergence();

        // just eye part is required
        lim=lim.submatrix(2,4,0,1);

        // read starting position
        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);
        getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
    }
    else
    {
        nJointsTorso=1;
        nJointsHead =5;

        // create bounds matrix for integrators
        // just for eye part
        lim.resize(3,2);

        // tilt
        lim(0,0)=(*chainEyeL)[nJointsTorso+2].getMin();
        lim(0,1)=(*chainEyeL)[nJointsTorso+2].getMax();

        // version
        lim(1,0)=(*chainEyeL)[nJointsTorso+3].getMin();
        lim(1,1)=(*chainEyeL)[nJointsTorso+3].getMax();

        // vergence
        lim(2,0)=commData->get_minAllowedVergence();
        lim(2,1)=lim(1,1);

        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);

        // impose starting vergence != 0.0
        fbHead[4]=commData->get_minAllowedVergence();
    }

    printf("lim:\n%s\n",lim.toString(10,3).c_str());

    // Instantiate integrator
    qd.resize(3);
    qd[0]=fbHead[2];
    qd[1]=fbHead[3];
    qd[2]=fbHead[4];
    I=new Integrator(Ts,qd,lim);

    gyro.resize(12,0.0);
    fp.resize(3,0.0);
    eyesJ.resize(3,3);
    eyesJ.zero();

    genOn=false;
    saccadeUnderWayOld=false;
    saccadesInhibitionPeriod=SACCADES_INHIBITION_PERIOD;
    port_xd=NULL;
}


/************************************************************************/
bool EyePinvRefGen::getGyro(Vector &data)
{
    if (port_inertial.getInputCount()>0)
    {
        mutex.wait();
        data=gyro;
        mutex.post();
        return true;
    }
    else
        return false;
}


/************************************************************************/
void EyePinvRefGen::setCounterRotGain(const Vector &gain)
{
    size_t len=std::min(counterRotGain.length(),gain.length());
    for (size_t i=0; i<len; i++)
        counterRotGain[i]=gain[i];
}


/************************************************************************/
Vector EyePinvRefGen::getEyesCounterVelocity(const Matrix &eyesJ, const Vector &fp)
{
    // ********** implement VOR
    Vector q(inertialSensor.getDOF());
    q[0]=fbTorso[0];
    q[1]=fbHead[0];
    q[2]=fbHead[1];

    Matrix H=inertialSensor.getH(q);

    H(0,3)=fp[0]-H(0,3);
    H(1,3)=fp[1]-H(1,3);
    H(2,3)=fp[2]-H(2,3);

    // gyro rate [deg/s]
    mutex.wait();
    double gyrX=gyro[6];
    double gyrY=gyro[7];
    double gyrZ=gyro[8];
    mutex.post();

    // filter out the noise on the gyro readouts
    Vector vor_fprelv;
    if ((fabs(gyrX)<GYRO_BIAS_STABILITY) && (fabs(gyrY)<GYRO_BIAS_STABILITY) &&
        (fabs(gyrZ)<GYRO_BIAS_STABILITY))
        vor_fprelv.resize(eyesJ.rows(),0.0);    // pinv(eyesJ) => use rows
    else
        vor_fprelv=CTRL_DEG2RAD*(gyrX*cross(H,0,H,3)+gyrY*cross(H,1,H,3)+gyrZ*cross(H,2,H,3));

    // ********** implement OCR
    Matrix H0=chainNeck->getH(1,true);
    Matrix H1=chainNeck->getH(2,true);
    Matrix H2=chainNeck->getH(3,true);

    for (int i=0; i<3; i++)
    {
        H0(i,3)=fp[i]-H0(i,3);
        H1(i,3)=fp[i]-H1(i,3);
        H2(i,3)=fp[i]-H2(i,3);
    }

    Vector v=commData->get_v();
    Vector ocr_fprelv=v[0]*cross(H0,2,H0,3)+v[1]*cross(H1,2,H1,3)+v[2]*cross(H2,2,H2,3);

    // ********** blend the contributions
    return -1.0*(pinv(eyesJ)*(counterRotGain[0]*vor_fprelv+counterRotGain[1]*ocr_fprelv));
}


/************************************************************************/
bool EyePinvRefGen::threadInit()
{
    string robotPortInertial=("/"+robotName+"/inertial");
    port_inertial.open((localName+"/inertial:i").c_str());
    if (!Network::connect(robotPortInertial.c_str(),port_inertial.getName().c_str()))
        fprintf(stdout,"Unable to connect to %s\n",robotPortInertial.c_str());

    fprintf(stdout,"Starting Pseudoinverse Reference Generator at %d ms\n",period);

    saccadesRxTargets=0;
    saccadesClock=Time::now();

    return true;
}


/************************************************************************/
void EyePinvRefGen::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Pseudoinverse Reference Generator started successfully\n");
    else
        fprintf(stdout,"Pseudoinverse Reference Generator did not start\n");
}


/************************************************************************/
void EyePinvRefGen::run()
{
    if (genOn)
    {
        double timeStamp;
        if (Robotable)
        {
            // read encoders
            getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData,&timeStamp);
            updateTorsoBlockedJoints(chainNeck,fbTorso);
            updateTorsoBlockedJoints(chainEyeL,fbTorso);
            updateTorsoBlockedJoints(chainEyeR,fbTorso);
        }
        else
        {
            fbHead=commData->get_q();
            timeStamp=Time::now();
        }

        // read gyro data
        if (Vector *_gyro=port_inertial.read(false))
        {
            mutex.wait();
            gyro=*_gyro;
            mutex.post();
        }

        // get current target
        Vector xd=port_xd->get_xd();

        // update neck chain
        chainNeck->setAng(nJointsTorso+0,fbHead[0]);
        chainNeck->setAng(nJointsTorso+1,fbHead[1]);


        // ask for saccades (if possible)
        if (Robotable && saccadesOn && (saccadesRxTargets!=port_xd->get_rx()) &&
            !commData->get_isSaccadeUnderway() && (Time::now()-saccadesClock>saccadesInhibitionPeriod))
        {
            Vector fph=xd; fph.push_back(1.0);
            fph=SE3inv(chainNeck->getH())*fph;

            // estimate geometrically the target tilt and pan of the eyes
            Vector ang(3,0.0);
            ang[0]=-atan2(fph[1],fabs(fph[2]));
            ang[1]=atan2(fph[0],fph[2]);

            // enforce joints bounds
            ang[0]=std::min(std::max(lim(0,0),ang[0]),lim(0,1));
            ang[1]=std::min(std::max(lim(1,0),ang[1]),lim(1,1));

            // favor the smooth-pursuit in case saccades are small
            if (norm(ang)>SACCADES_ACTIVATIONANGLE*CTRL_DEG2RAD)
            {
                // init vergence
                ang[2]=fbHead[4];

                // get rid of eyes tilt
                Vector axis(4);
                axis[0]=1.0; axis[1]=0.0; axis[2]=0.0; axis[3]=-ang[0];
                fph=axis2dcm(axis)*fph;

                // go on iff the point is in front of us
                if (fph[2]>0.0)
                {
                    double L,R;

                    // estimate geometrically the target vergence Vg=L-R                    
                    if (fph[0]>=eyesHalfBaseline)
                    {
                        L=M_PI/2.0-atan2(fph[2],fph[0]+eyesHalfBaseline);
                        R=M_PI/2.0-atan2(fph[2],fph[0]-eyesHalfBaseline);
                    }
                    else if (fph[0]>-eyesHalfBaseline)
                    {
                        L=M_PI/2.0-atan2(fph[2],fph[0]+eyesHalfBaseline);
                        R=-(M_PI/2.0-atan2(fph[2],eyesHalfBaseline-fph[0]));
                    }
                    else
                    {
                        L=-(M_PI/2.0-atan2(fph[2],-fph[0]-eyesHalfBaseline));
                        R=-(M_PI/2.0-atan2(fph[2],eyesHalfBaseline-fph[0]));
                    }

                    ang[2]=L-R;
                }

                // enforce joints bounds
                ang[2]=std::min(std::max(lim(2,0),ang[2]),lim(2,1));

                commData->set_qd(2,ang[0]);
                commData->set_qd(3,ang[1]);
                commData->set_qd(4,ang[2]);

                Vector vel(3,SACCADES_VEL);
                ctrl->doSaccade(ang,vel);
                saccadesClock=Time::now();
            }
        }

        // update eyes chains for convergence purpose
        updateNeckBlockedJoints(chainEyeL,fbHead);         updateNeckBlockedJoints(chainEyeR,fbHead);
        chainEyeL->setAng(nJointsTorso+2,qd[0]);           chainEyeR->setAng(nJointsTorso+2,qd[0]);
        //chainEyeL->setAng(nJointsTorso+3,qd[1]+qd[2]/2.0); chainEyeR->setAng(nJointsTorso+3,qd[1]-qd[2]/2.0);
	chainEyeL->setAng(nJointsTorso+3,(qd[1]+qd[2])/2.0); chainEyeR->setAng(nJointsTorso+3,(qd[1]-qd[2])/2.0);


        // converge on target
        if (computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ))
        {

            Vector v=EYEPINVREFGEN_GAIN*(pinv(eyesJ)*(xd-fp));

            // update eyes chains in actual configuration for velocity compensation
            chainEyeL->setAng(nJointsTorso+2,fbHead[2]);               chainEyeR->setAng(nJointsTorso+2,fbHead[2]);
            //chainEyeL->setAng(nJointsTorso+3,fbHead[3]+fbHead[4]/2.0); chainEyeR->setAng(nJointsTorso+3,fbHead[3]-fbHead[4]/2.0);
	    chainEyeL->setAng(nJointsTorso+3,(fbHead[3]+fbHead[4])/2.0); chainEyeR->setAng(nJointsTorso+3,(fbHead[3]-fbHead[4])/2.0);

            // compensate neck rotation at eyes level
            if (computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ))
                commData->set_counterv(getEyesCounterVelocity(eyesJ,fp));
            else
                commData->set_counterv(zeros(3));

            // reset eyes controller and integral upon saccades transition on=>off
            if (saccadeUnderWayOld && !commData->get_isSaccadeUnderway())
            {
                ctrl->resetCtrlEyes();

                qd[0]=fbHead[2];
                qd[1]=fbHead[3];
                qd[2]=fbHead[4];
                I->reset(qd);
            }

            // update reference
            qd=I->integrate(v+commData->get_counterv());
        }
        else
            commData->set_counterv(zeros(3));

        // set a new target position
        commData->set_xd(xd);
        commData->set_x(fp,timeStamp);
        commData->set_fpFrame(chainNeck->getH());
        if (!commData->get_isSaccadeUnderway())
        {
            commData->set_qd(2,qd[0]);
            commData->set_qd(3,qd[1]);
            commData->set_qd(4,qd[2]);
        }

        // latch the saccades status
        saccadeUnderWayOld=commData->get_isSaccadeUnderway();
        saccadesRxTargets=port_xd->get_rx();
    }
}


/************************************************************************/
void EyePinvRefGen::threadRelease()
{
    port_inertial.interrupt();
    port_inertial.close();

    delete neck;
    delete eyeL;
    delete eyeR;
    delete I;
}


/************************************************************************/
void EyePinvRefGen::suspend()
{
    fprintf(stdout,"\nPseudoinverse Reference Generator has been suspended!\n\n");
    RateThread::suspend();
}


/************************************************************************/
void EyePinvRefGen::resume()
{
    fprintf(stdout,"\nPseudoinverse Reference Generator has been resumed!\n\n");
    RateThread::resume();
}


/************************************************************************/
void EyePinvRefGen::stopControl()
{
    port_xd->set_xd(fp);
}


/************************************************************************/
Solver::Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
               EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
               const string &_localName, ResourceFinder &_camerasFile, const double _eyeTiltMin,
               const double _eyeTiltMax, const bool _headV2,
               const string &_root_link, const unsigned int _period) :
               RateThread(_period),     drvTorso(_drvTorso),     drvHead(_drvHead),
               commData(_commData),     eyesRefGen(_eyesRefGen), loc(_loc),
               ctrl(_ctrl),             localName(_localName),
               eyeTiltMin(_eyeTiltMin), eyeTiltMax(_eyeTiltMax), headV2(_headV2),
               period(_period),         Ts(_period/1000.0)
{
    this->rf_camera=_camerasFile;
    Robotable=(drvHead!=NULL);

    // Instantiate objects
    neck=new vizzyHeadCenter(headV2?"right_v2":"right",_root_link);
    eyeL=new vizzyEye(headV2?"left_v2":"left",_root_link);
    eyeR=new vizzyEye(headV2?"right_v2":"right",_root_link);

    // remove constraints on the links: logging purpose
    inertialSensor.setAllConstraints(false);

    // block neck dofs
    eyeL->blockLink(1,0.0); eyeR->blockLink(1,0.0);
    eyeL->blockLink(2,0.0); eyeR->blockLink(2,0.0);
    //eyeL->blockLink(4,0.0); eyeR->blockLink(4,0.0);

    // Get the chain objects
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();        
    chainEyeR=eyeR->asChain();

    /*for(int i =0; i < chainEyeL->getN(); i++){
        	printf("eyeL[%d]=(%.3f,%.3f,%.3f,%.3f)\n",i,(*chainEyeL)[i].getA(),(*chainEyeL)[i].getD(),(*chainEyeL)[i].getAlpha(),(*chainEyeL)[i].getOffset());
        }
    for(int i =0; i < chainEyeR->getN(); i++){
        	printf("eyeR[%d]=(%.3f,%.3f,%.3f,%.3f)\n",i,(*chainEyeR)[i].getA(),(*chainEyeR)[i].getD(),(*chainEyeR)[i].getAlpha(),(*chainEyeR)[i].getOffset());
        }

        printf("HL(%s)\n",chainEyeL->getH().toString(3,3).c_str());
        printf("HR(%s)\n",chainEyeR->getH().toString(3,3).c_str());*/

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

        // joints bounds alignment
        alignJointsBounds(chainNeck,drvTorso,drvHead,eyeTiltMin,eyeTiltMax);
        copyJointsBounds(chainNeck,chainEyeL);
        copyJointsBounds(chainEyeL,chainEyeR);

        // read starting position
        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);
        getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
    }
    else
    {
        nJointsTorso=1;
        nJointsHead =5;

        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);

        // impose starting vergence != 0.0
        fbHead[5]=commData->get_minAllowedVergence();
    }

    // store neck pitch/yaw bounds
    neckYawMin  =(*chainNeck)[0].getMin();
    neckYawMax  =(*chainNeck)[0].getMax();
    neckPitchMin=(*chainNeck)[1].getMin();
    neckPitchMax=(*chainNeck)[1].getMax();

    neckPos.resize(3);
    gazePos.resize(3);
    updateAngles();

    updateTorsoBlockedJoints(chainNeck,fbTorso);
    chainNeck->setAng(2,fbHead[0]);
    chainNeck->setAng(3,fbHead[1]);
    chainNeck->setAng(4,fbHead[2]);

    updateTorsoBlockedJoints(chainEyeL,fbTorso);
    updateTorsoBlockedJoints(chainEyeR,fbTorso);
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);

    Vector eyePos(2);
    eyePos[0]=gazePos[0];
    //eyePos[1]=gazePos[1]+gazePos[2]/2.0;
    eyePos[1]=(gazePos[1]+gazePos[2])/2.0;
    chainEyeL->setAng(eyePos);
    //eyePos[1]=gazePos[1]-gazePos[2]/2.0;
    eyePos[1]=(gazePos[1]-gazePos[2])/2.0;
    chainEyeR->setAng(eyePos);

    gDefaultDir.resize(4,0.0);
    gDefaultDir[2]=gDefaultDir[3]=1.0;

    fbTorsoOld=fbTorso;
    fbHeadOld=fbHead;

    bindSolveRequest=false;
}


/************************************************************************/
void Solver::bindNeckPitch(const double min_deg, const double max_deg)
{
    double min_rad=sat(CTRL_DEG2RAD*min_deg,neckPitchMin,neckPitchMax);
    double max_rad=sat(CTRL_DEG2RAD*max_deg,neckPitchMin,neckPitchMax);
    double cur_rad=(*chainNeck)(1).getAng();

    bindSolveRequest=(cur_rad<min_rad) || (cur_rad>max_rad);

    mutex.wait();
    (*chainNeck)(1).setMin(min_rad);
    (*chainNeck)(1).setMax(max_rad);
    mutex.post();

    fprintf(stdout,"\nneck pitch constrained in [%g,%g] deg\n\n",min_deg,max_deg);
}


/************************************************************************/
void Solver::bindNeckYaw(const double min_deg, const double max_deg)
{
    double min_rad=sat(CTRL_DEG2RAD*min_deg,neckYawMin,neckYawMax);
    double max_rad=sat(CTRL_DEG2RAD*max_deg,neckYawMin,neckYawMax);
    double cur_rad=(*chainNeck)(0).getAng();

    bindSolveRequest=(cur_rad<min_rad) || (cur_rad>max_rad);

    mutex.wait();
    (*chainNeck)(0).setMin(min_rad);
    (*chainNeck)(0).setMax(max_rad);
    mutex.post();

    fprintf(stdout,"\nneck yaw constrained in [%g,%g] deg\n\n",min_deg,max_deg);
}


/************************************************************************/
void Solver::getCurNeckPitchRange(double &min_deg, double &max_deg)
{
    mutex.wait();
    min_deg=CTRL_RAD2DEG*(*chainNeck)(1).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(1).getMax();
    mutex.post();
}


/************************************************************************/
void Solver::getCurNeckYawRange(double &min_deg, double &max_deg)
{
    mutex.wait();
    min_deg=CTRL_RAD2DEG*(*chainNeck)(0).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(0).getMax();
    mutex.post();
}


/************************************************************************/
void Solver::clearNeckPitch()
{
    mutex.wait();
    (*chainNeck)(1).setMin(neckPitchMin);
    (*chainNeck)(1).setMax(neckPitchMax);
    mutex.post();

    fprintf(stdout,"\nneck pitch cleared\n\n");
}


/************************************************************************/
void Solver::clearNeckYaw()
{
    mutex.wait();
    (*chainNeck)(0).setMin(neckYawMin);
    (*chainNeck)(0).setMax(neckYawMax);
    mutex.post();

    fprintf(stdout,"\nneck yaw cleared\n\n");
}


/************************************************************************/
void Solver::updateAngles()
{
    neckPos[0]=fbHead[0];
    neckPos[1]=fbHead[1];
    gazePos[0]=fbHead[2];
    gazePos[1]=fbHead[3];
    gazePos[2]=fbHead[4];
}


/************************************************************************/
Vector Solver::getGravityDirection(const Vector &gyro)
{
    double roll =CTRL_DEG2RAD*gyro[0];
    double pitch=CTRL_DEG2RAD*gyro[1];

    // compute rotational matrix to
    // account for roll and pitch
    Vector x(4), y(4);
    x[0]=1.0;    y[0]=0.0;
    x[1]=0.0;    y[1]=1.0;
    x[2]=0.0;    y[2]=0.0;
    x[3]=roll;   y[3]=pitch;   
    Matrix R=axis2dcm(y)*axis2dcm(x);

    Vector q(inertialSensor.getDOF());
    q[0]=fbTorso[0];
    q[1]=fbHead[0];
    q[2]=fbHead[1];
    Matrix H=inertialSensor.getH(q)*R.transposed();

    // gravity is aligned along the z-axis
    Vector gDir=H.getCol(2);
    gDir[3]=1.0;    // impose homogeneous coordinates

    return gDir;
}


/************************************************************************/
Vector Solver::neckTargetRotAngles(const Vector &xd)
{
    Vector fph=xd; fph.push_back(1.0);
    fph=SE3inv(commData->get_fpFrame())*fph;

    Vector ang(2);
    ang[0]=-atan2(fph[1],fabs(fph[2]));
    ang[1]=atan2(fph[0],fph[2]);

    return ang;
}


/************************************************************************/
bool Solver::threadInit()
{
    // Instantiate optimizer
    invNeck=new GazeIpOptMin(*chainNeck,1e-3,20);

    // Initialization
    Vector fp(3);
    Matrix J(3,3);
    computeFixationPointData(*chainEyeL,*chainEyeR,fp,J);

    // init commData structure
    commData->set_xd(fp);
    commData->set_qd(fbHead);
    commData->set_x(fp);
    commData->set_q(fbHead);
    commData->set_torso(fbTorso);
    commData->resize_v(fbHead.length(),0.0);
    commData->resize_counterv(3,0.0);
    commData->set_fpFrame(chainNeck->getH());

    port_xd=new xdPort(fp,this);
    port_xd->useCallback();
    port_xd->open((localName+"/xd:i").c_str());

    loc->set_xdport(port_xd);
    eyesRefGen->set_xdport(port_xd);
    ctrl->set_xdport(port_xd);

    // use eyes pseudoinverse reference generator
    eyesRefGen->enable();

    fprintf(stdout,"Starting Solver at %d ms\n",period);

    return true;
}


/************************************************************************/
void Solver::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Solver started successfully\n");
    else
        fprintf(stdout,"Solver did not start\n");
}


/************************************************************************/
void Solver::run()
{
    mutex.wait();

    // get the current target
    Vector xd=port_xd->get_xdDelayed();

    // update the target straightaway 
    commData->set_xd(xd);

    bool torsoChanged=false;

    if (Robotable)
    {
        // read encoders
        getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
        updateTorsoBlockedJoints(chainNeck,fbTorso);
        updateTorsoBlockedJoints(chainEyeL,fbTorso);
        updateTorsoBlockedJoints(chainEyeR,fbTorso);

        torsoChanged=norm(fbTorso-fbTorsoOld)>NECKSOLVER_ACTIVATIONANGLE_JOINTS*CTRL_DEG2RAD;        
    }
    else
        fbHead=commData->get_q();

    bool headChanged=norm(fbHead-fbHeadOld)>NECKSOLVER_ACTIVATIONANGLE_JOINTS*CTRL_DEG2RAD;

    // update kinematics
    updateAngles();
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);
    chainNeck->setAng(neckPos);

    // hereafter accumulate solving conditions: the order does matter

    // 1) compute the distance in the transverse and sagittal planes
    Vector theta=neckTargetRotAngles(xd);
    bool doSolve=(norm(theta)>NECKSOLVER_ACTIVATIONANGLE*CTRL_DEG2RAD);

    // 2) skip if controller is active and no torso motion is detected
    doSolve&=!(commData->get_isCtrlActive() && !torsoChanged);

    // 3) skip if controller is inactive and we are not in tracking mode
    doSolve&=!(!commData->get_isCtrlActive() && commData->get_canCtrlBeDisabled());

    // 4) skip if controller is inactive, we are in tracking mode and nothing has changed
    // note that the De Morgan's law has been applied hereafter
    doSolve&=commData->get_isCtrlActive() || commData->get_canCtrlBeDisabled() ||
             torsoChanged || headChanged;

    // 5) solve straightaway if we are in tracking mode and a request is raised
    // by the binding methods
    doSolve|=!commData->get_canCtrlBeDisabled() && bindSolveRequest;

    // 6) solve straightaway if the target has changed
    doSolve|=port_xd->get_newDelayed();

    // clear triggers
    port_xd->get_newDelayed()=false;
    bindSolveRequest=false;

    // call the solver for neck
    if (doSolve)
    {
        Vector  gyro;
        Vector  gDir;
        Vector *pgDir;

        if (eyesRefGen->getGyro(gyro))
        {
            gDir=getGravityDirection(gyro);
            pgDir=&gDir;
        }
        else
            pgDir=&gDefaultDir;

        neckPos=invNeck->solve(neckPos,xd,*pgDir);

        // update neck pitch,roll,yaw        
        commData->set_qd(0,neckPos[0]);
        commData->set_qd(1,neckPos[1]);
    }

    // latch quantities
    fbTorsoOld=fbTorso;
    fbHeadOld=fbHead;

    mutex.post();
}


/************************************************************************/
void Solver::threadRelease()
{
    eyesRefGen->disable();

    port_xd->interrupt();
    port_xd->close();

    delete port_xd;
    delete invNeck;
    delete neck;
    delete eyeL;
    delete eyeR;
}


/************************************************************************/
void Solver::suspend()
{
    fprintf(stdout,"\nSolver has been suspended!\n\n");
    RateThread::suspend();
}


/************************************************************************/
void Solver::resume()
{
    mutex.wait();

    if (Robotable)
    {
        // read encoders
        getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
        updateTorsoBlockedJoints(chainNeck,fbTorso);
        updateTorsoBlockedJoints(chainEyeL,fbTorso);
        updateTorsoBlockedJoints(chainEyeR,fbTorso);        
    }
    else
        fbHead=commData->get_q();    

    // update kinematics
    updateAngles();
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);
    chainNeck->setAng(neckPos);
    chainEyeL->setAng(nJointsTorso+2,gazePos[0]);                chainEyeR->setAng(nJointsTorso+2,gazePos[0]);
    //chainEyeL->setAng(nJointsTorso+3,gazePos[1]+gazePos[2]/2.0); chainEyeR->setAng(nJointsTorso+3,gazePos[1]-gazePos[2]/2.0);
    chainEyeL->setAng(nJointsTorso+3,(gazePos[1]+gazePos[2])/2.0); chainEyeR->setAng(nJointsTorso+3,(gazePos[1]-gazePos[2])/2.0);

    // compute fixation point
    Vector fp(3);
    Matrix J(3,3);
    computeFixationPointData(*chainEyeL,*chainEyeR,fp,J);

    // update input port
    port_xd->set_xd(fp);

    // update latched quantities
    fbTorsoOld=fbTorso;
    fbHeadOld=fbHead;    

    fprintf(stdout,"\nSolver has been resumed!\n\n");

    mutex.post();

    RateThread::resume();
}



