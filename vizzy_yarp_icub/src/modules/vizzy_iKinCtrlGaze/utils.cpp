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
#include <yarp/dev/ControlBoardInterfaces.h>
#include <vizzy/utils.h>
#include <vizzy/solver.h>

#define MUTEX_XD            0
#define MUTEX_QD            1
#define MUTEX_X             2
#define MUTEX_Q             3
#define MUTEX_TORSO         4
#define MUTEX_V             5
#define MUTEX_COUNTERV      6
#define MUTEX_FPFRAME       7


/************************************************************************/
xdPort::xdPort(const Vector &xd0, void *_slv)
{   
    xdDelayed=xd=xd0;
    isNewDelayed=isNew=false;
    closing=false;
    rx=0;

    slv=_slv;
    if (slv!=NULL)
        start();
}


/************************************************************************/
xdPort::~xdPort()
{
    closing=true;
    syncEvent.signal();

    if (slv!=NULL)
        stop();
}


/************************************************************************/
void xdPort::onRead(Bottle &b)
{
    mutex_0.wait();

    int bLen=b.size();
    int xdLen=xd.length();
    int n=bLen>xdLen ? xdLen : bLen;    

    for (int i=0; i<n; i++)
        xd[i]=b.get(i).asDouble();

    isNew=true;
    rx++;

    mutex_0.post();

    syncEvent.signal();
}


/************************************************************************/
void xdPort::set_xd(const Vector &_xd)
{
    mutex_0.wait();
    xd=_xd;
    isNew=true;
    mutex_0.post();
    syncEvent.signal();
}


/************************************************************************/
Vector xdPort::get_xd()
{
    mutex_0.wait();
    Vector _xd=xd;
    mutex_0.post();
    return _xd;
}


/************************************************************************/
Vector xdPort::get_xdDelayed()
{
    mutex_1.wait();
    Vector _xdDelayed=xdDelayed;
    mutex_1.post();
    return _xdDelayed;
}


/************************************************************************/
void xdPort::run()
{
    while (!isStopping() && !closing)
    {
        syncEvent.reset();
        syncEvent.wait();

        Vector theta=static_cast<Solver*>(slv)->neckTargetRotAngles(xd);

        double timeDelay=0.0;
        if (norm(theta)<NECKSOLVER_RESTORINGANGLE*CTRL_DEG2RAD)
            timeDelay=NECKSOLVER_ACTIVATIONDELAY;

        Time::delay(timeDelay);

        mutex_1.wait();

        xdDelayed=xd;
        isNewDelayed=true;

        mutex_1.post();
    }
}


/************************************************************************/
exchangeData::exchangeData()
{
    isCtrlActive=false;
    canCtrlBeDisabled=true;
    saccadeUnderway=false;
    minAllowedVergence=0.0;
}


/************************************************************************/
void exchangeData::resize_v(const int sz, const double val)
{
    mutex[MUTEX_V].wait();
    v.resize(sz,val);
    mutex[MUTEX_V].post();
}


/************************************************************************/
void exchangeData::resize_counterv(const int sz, const double val)
{
    mutex[MUTEX_COUNTERV].wait();
    counterv.resize(sz,val);
    mutex[MUTEX_COUNTERV].post();
}


/************************************************************************/
void exchangeData::set_xd(const Vector &_xd)
{
    mutex[MUTEX_XD].wait();
    xd=_xd;
    mutex[MUTEX_XD].post();
}


/************************************************************************/
void exchangeData::set_qd(const Vector &_qd)
{
    mutex[MUTEX_QD].wait();
    qd=_qd;
    mutex[MUTEX_QD].post();
}


/************************************************************************/
void exchangeData::set_qd(const int i, const double val)
{
    mutex[MUTEX_QD].wait();
    qd[i]=val;
    mutex[MUTEX_QD].post();
}


/************************************************************************/
void exchangeData::set_x(const Vector &_x)
{
    mutex[MUTEX_X].wait();
    x=_x;
    mutex[MUTEX_X].post();
}


/************************************************************************/
void exchangeData::set_x(const Vector &_x, const double stamp)
{
    mutex[MUTEX_X].wait();
    x=_x;
    x_stamp=stamp;
    mutex[MUTEX_X].post();
}


/************************************************************************/
void exchangeData::set_q(const Vector &_q)
{
    mutex[MUTEX_Q].wait();
    q=_q;
    mutex[MUTEX_Q].post();
}


/************************************************************************/
void exchangeData::set_torso(const Vector &_torso)
{
    mutex[MUTEX_TORSO].wait();
    torso=_torso;
    mutex[MUTEX_TORSO].post();
}


/************************************************************************/
void exchangeData::set_v(const Vector &_v)
{
    mutex[MUTEX_V].wait();
    v=_v;
    mutex[MUTEX_V].post();
}


/************************************************************************/
void exchangeData::set_counterv(const Vector &_counterv)
{
    mutex[MUTEX_COUNTERV].wait();
    counterv=_counterv;
    mutex[MUTEX_COUNTERV].post();
}


/************************************************************************/
void exchangeData::set_fpFrame(const Matrix &_S)
{
    mutex[MUTEX_FPFRAME].wait();
    S=_S;
    mutex[MUTEX_FPFRAME].post();
}


/************************************************************************/
Vector exchangeData::get_xd()
{
    mutex[MUTEX_XD].wait();
    Vector _xd=xd;
    mutex[MUTEX_XD].post();

    return _xd;
}


/************************************************************************/
Vector exchangeData::get_qd()
{
    mutex[MUTEX_QD].wait();
    Vector _qd=qd;
    mutex[MUTEX_QD].post();

    return _qd;
}


/************************************************************************/
Vector exchangeData::get_x()
{
    mutex[MUTEX_X].wait();
    Vector _x=x;
    mutex[MUTEX_X].post();

    return _x;
}


/************************************************************************/
Vector exchangeData::get_x(double &stamp)
{
    mutex[MUTEX_X].wait();
    Vector _x=x;
    stamp=x_stamp;
    mutex[MUTEX_X].post();

    return _x;
}


/************************************************************************/
Vector exchangeData::get_q()
{
    mutex[MUTEX_Q].wait();
    Vector _q=q;
    mutex[MUTEX_Q].post();

    return _q;
}


/************************************************************************/
Vector exchangeData::get_torso()
{
    mutex[MUTEX_TORSO].wait();
    Vector _torso=torso;
    mutex[MUTEX_TORSO].post();

    return _torso;
}


/************************************************************************/
Vector exchangeData::get_v()
{
    mutex[MUTEX_V].wait();
    Vector _v=v;
    mutex[MUTEX_V].post();

    return _v;
}


/************************************************************************/
Vector exchangeData::get_counterv()
{
    mutex[MUTEX_COUNTERV].wait();
    Vector _counterv=counterv;
    mutex[MUTEX_COUNTERV].post();

    return _counterv;
}


/************************************************************************/
Matrix exchangeData::get_fpFrame()
{
    mutex[MUTEX_FPFRAME].wait();
    Matrix _S=S;
    mutex[MUTEX_FPFRAME].post();

    return _S;
}


/************************************************************************/
bool getCamPrj(const ResourceFinder &rf, const string &type, Matrix **Prj)
{
    ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
    *Prj=NULL;
    if (!_rf.isConfigured())
        return false;

    string camerasFile=_rf.findFile("from").c_str();
    if (!camerasFile.empty())
    {
        Bottle parType=_rf.findGroup(type.c_str());
        string warning="Intrinsic parameters for "+type+" group not found";

            if (parType.check("cx") && parType.check("cy") &&
                parType.check("fx") && parType.check("fy"))
            {
                double cx=parType.find("cx").asDouble();
                double cy=parType.find("cy").asDouble();
                double fx=parType.find("fx").asDouble();
                double fy=parType.find("fy").asDouble();

                Matrix K=eye(3,3);
                Matrix Pi=zeros(3,4);

                K(0,0)=fx; K(1,1)=fy;
                K(0,2)=cx; K(1,2)=cy; 
                
                Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 

                *Prj=new Matrix;
                **Prj=K*Pi;
                return true;
            }
            else
                fprintf(stdout,"%s\n",warning.c_str());
    }
    return false;
}


/***********************************************************************
bool getAlignHN(const string &camerasFile, const string &type, iKinChain *chain)
{
    if (chain!=NULL)
    {
        Property par;
        string warning="Aligning matrix for "+type+" group not found";
        if (par.fromConfigFile(camerasFile.c_str()))
        {
            Bottle parType=par.findGroup(type.c_str());
            if (parType.size()>0)
            {
                if (Bottle *bH=parType.find("HN").asList())
                {
                    int i=0;
                    int j=0;

                    Matrix HN(4,4); HN=0.0;
                    for (int cnt=0; (cnt<bH->size()) && (cnt<HN.rows()*HN.cols()); cnt++)
                    {    
                        HN(i,j)=bH->get(cnt).asDouble();
                        if (++j>=HN.cols())
                        {
                            i++;
                            j=0;
                        }
                    }

                    // enforce the homogeneous property
                    HN(3,0)=HN(3,1)=HN(3,2)=0.0;
                    HN(3,3)=1.0;

                    chain->setHN(HN);
                    return true;
                }
            }
        }

        fprintf(stdout,"%s\n",warning.c_str());
    }

    return false;
}
***********************************************/

/************************************************************************/
bool getAlignHN(const ResourceFinder &rf, const string &type,
                iKinChain *chain, const bool verbose)
{
    ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
    if ((chain!=NULL) && _rf.isConfigured())
    {
        string message=_rf.findFile("from").c_str();
        if (!message.empty())
        {
            message+=": aligning matrix for "+type;
            Bottle &parType=_rf.findGroup(type.c_str());
            if (!parType.isNull())
            {
                if (Bottle *bH=parType.find("HN").asList())
                {
                    int i=0;
                    int j=0;

                    Matrix HN(4,4); HN=0.0;
                    for (int cnt=0; (cnt<bH->size()) && (cnt<HN.rows()*HN.cols()); cnt++)
                    {
                        HN(i,j)=bH->get(cnt).asDouble();
                        if (++j>=HN.cols())
                        {
                            i++;
                            j=0;
                        }
                    }

                    // enforce the homogeneous property
                    HN(3,0)=HN(3,1)=HN(3,2)=0.0;
                    HN(3,3)=1.0;

                    chain->setHN(HN);

                    if (verbose)
                    {
                        fprintf(stdout,"%s found:",message.c_str());
                        fprintf(stdout,"%s",HN.toString(3,3).c_str());
                    }

                    return true;
                }
            }
        }
        else
        {
            message=_rf.find("from").asString().c_str();
            message+=": aligning matrix for "+type;
        }

        if (verbose)
            fprintf(stdout,"%s not found!",message.c_str());
    }

    return false;
}




/************************************************************************/
Matrix alignJointsBounds(iKinChain *chain, PolyDriver *drvTorso, PolyDriver *drvHead,
                         const double eyeTiltMin, const double eyeTiltMax)
{
    IEncoders      *encs;
    IControlLimits *lims;

    double min, max;
    int nJointsTorso=1;

    if (drvTorso!=NULL)
    {
        drvTorso->view(encs);
        drvTorso->view(lims);        
        encs->getAxes(&nJointsTorso);

        for (int i=0; i<nJointsTorso; i++)
        {   
            lims->getLimits(i,&min,&max);
        
            (*chain)[nJointsTorso-1-i].setMin(CTRL_DEG2RAD*min); // reversed order
            (*chain)[nJointsTorso-1-i].setMax(CTRL_DEG2RAD*max);
        }
    }

    drvHead->view(encs);
    drvHead->view(lims);
    int nJointsHead;
    encs->getAxes(&nJointsHead);
    Matrix lim(nJointsHead,2);

    for (int i=0; i<nJointsHead; i++)
    {   
        lims->getLimits(i,&min,&max);

        // limit eye's tilt due to eyelids
        if (i==2)
        {
            min=std::max(min,eyeTiltMin);
            max=std::min(max,eyeTiltMax);
        }

        lim(i,0)=CTRL_DEG2RAD*min;
        lim(i,1)=CTRL_DEG2RAD*max;

        // just one eye's got only 5 dofs
        if (i<nJointsHead-1)
        {
            (*chain)[nJointsTorso+i].setMin(lim(i,0));
            (*chain)[nJointsTorso+i].setMax(lim(i,1));
        }
    }

    return lim;
}


/************************************************************************/
void copyJointsBounds(iKinChain *ch1, iKinChain *ch2)
{
    unsigned int N1=ch1->getN();
    unsigned int N2=ch2->getN();
    unsigned int N =N1>N2 ? N2 : N1;

    for (unsigned int i=0; i<N; i++)
    {
        (*ch2)[i].setMin((*ch1)[i].getMin());
        (*ch2)[i].setMax((*ch1)[i].getMax());
    }
}


/************************************************************************/
void updateTorsoBlockedJoints(iKinChain *chain, const Vector &fbTorso)
{
    for (size_t i=0; i<fbTorso.length(); i++)
         chain->setBlockingValue(i,fbTorso[i]);
}


/************************************************************************/
void updateNeckBlockedJoints(iKinChain *chain, const Vector &fbNeck)
{
    for (int i=0; i<2; i++)
         chain->setBlockingValue(1+i,fbNeck[i]);
}


/************************************************************************/
bool getFeedback(Vector &fbTorso, Vector &fbHead, PolyDriver *drvTorso,
                 PolyDriver *drvHead, exchangeData *commData, double *timeStamp)
{
    IEncodersTimed *encs;

    int nJointsTorso=fbTorso.length();
    int nJointsHead=fbHead.length();

    Vector fb(std::max(nJointsTorso,nJointsHead));
    Vector stamps(nJointsTorso+nJointsHead,0.0);
    bool ret=true;
    
    if (drvTorso!=NULL)
    {
        drvTorso->view(encs);
        if (encs->getEncodersTimed(fb.data(),stamps.data()))
        {
            for (int i=0; i<nJointsTorso; i++)
                fbTorso[i]=CTRL_DEG2RAD*fb[nJointsTorso-1-i];    // reversed order
        }
        else
            ret=false;
    }
    else
        fbTorso=0.0;

    drvHead->view(encs);
    if (encs->getEncodersTimed(fb.data(),stamps.data()+nJointsTorso))
    {
        for (int i=0; i<nJointsHead; i++){
            fbHead[i]=CTRL_DEG2RAD*fb[i];
        }
    }
    else
        ret=false;
    
    // impose vergence != 0.0
    if (commData!=NULL)
        if (fbHead[nJointsHead-1]<commData->get_minAllowedVergence())
            fbHead[nJointsHead-1]=commData->get_minAllowedVergence();

    // retrieve the highest encoders time stamp
    if (timeStamp!=NULL)
        *timeStamp=findMax(stamps);

    return ret;
}


