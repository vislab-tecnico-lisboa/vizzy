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

#include <stdio.h>

#include <yarp/math/SVD.h>
#include <vizzy/localizer.h>
#include <vizzy/solver.h>

/*Localizer::Localizer(exchangeData *_commData, const unsigned int _period) :
RateThread(_period), commData(_commData), period(_period)*/

/************************************************************************/
Localizer::Localizer(exchangeData *_commData, const string &_localName,
                     ResourceFinder &_camerasFile, const bool _headV2,
                     const string &_root_link, const unsigned int _period) :
                     RateThread(_period), commData(_commData), localName(_localName),
                     headV2(_headV2),  period(_period)
{
    this->rf_camera=_camerasFile;
    vizzyHeadCenter eyeC(headV2?"right_v2":"right",_root_link);
    eyeL=new vizzyEye(headV2?"left_v2":"left",_root_link);
    eyeR=new vizzyEye(headV2?"right_v2":"right",_root_link);

    // remove constraints on the links
    // we use the chains for logging purpose
    eyeL->setAllConstraints(false);
    eyeR->setAllConstraints(false);

    // release links
    eyeL->releaseLink(0); eyeC.releaseLink(0); eyeR->releaseLink(0);
    eyeL->releaseLink(1); eyeC.releaseLink(1); eyeR->releaseLink(1);
    eyeL->releaseLink(2); /*eyeC.releaseLink(2);*/ eyeR->releaseLink(2);

    // add aligning matrices read from configuration file
    getAlignHN(rf_camera,"ALIGN_KIN_LEFT",eyeL->asChain(),true);
    getAlignHN(rf_camera,"ALIGN_KIN_RIGHT",eyeR->asChain(),true);

    // get the absolute reference frame of the head
    Vector q(eyeC.getDOF(),0.0);
    eyeCAbsFrame=eyeC.getH(q);
    // ... and its inverse
    invEyeCAbsFrame=SE3inv(eyeCAbsFrame);

    // get the lenght of the half of the eyes baseline
    eyesHalfBaseline=0.5*norm(eyeL->EndEffPose().subVector(0,2)-eyeR->EndEffPose().subVector(0,2));

    // get camera projection matrix from the camerasFile
    if (getCamPrj(rf_camera,"CAMERA_CALIBRATION_LEFT",&PrjL))
    {
        Matrix &Prj=*PrjL;
        cxl=Prj(0,2);
        cyl=Prj(1,2);

        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else
        PrjL=invPrjL=NULL;

    // get camera projection matrix from the camerasFile
    if (getCamPrj(rf_camera,"CAMERA_CALIBRATION_RIGHT",&PrjR))
    {
        Matrix &Prj=*PrjR;
        cxr=Prj(0,2);
        cyr=Prj(1,2);

        invPrjR=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else
        PrjR=invPrjR=NULL;

    Vector Kp(1,0.001), Ki(1,0.001), Kd(1,0.0);
    Vector Wp(1,1.0),   Wi(1,1.0),   Wd(1,1.0);
    Vector N(1,10.0),   Tt(1,1.0);
    Matrix satLim(1,2);

    satLim(0,0)=0.05;
    satLim(0,1)=10.0;

    pid=new parallelPID(0.05,Kp,Ki,Kd,Wp,Wi,Wd,N,Tt,satLim);

    Vector z0(1,0.5);
    pid->reset(z0);
    dominantEye="left";

    port_xd=NULL;
}


/************************************************************************/
bool Localizer::threadInit()
{ 
    port_mono.open((localName+"/mono:i").c_str());
    port_stereo.open((localName+"/stereo:i").c_str());
    port_anglesIn.open((localName+"/angles:i").c_str());
    port_anglesOut.open((localName+"/angles:o").c_str());
    /*ros_port_anglesOut.setWriteOnly();
    ros_port_anglesOut.open("/vizzy_gaze_controller/angles_out@/vizzy_iKinGazeCtrl_ros");
    while(ros_port_anglesOut.getOutputCount() == 0) {
        Time::delay(1);
        std::cout << ".\n";
    }
    std::cout << "Connection successfuly established." << std::endl;*/
    //Time::delay(5);
    fprintf(stdout,"Starting Localizer at %d ms\n",period);

    return true;
}


/************************************************************************/
void Localizer::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Localizer started successfully\n");
    else
        fprintf(stdout,"Localizer did not start\n");
}


/************************************************************************/
void Localizer::getPidOptions(Bottle &options)
{
    mutex.wait();

    pid->getOptions(options);
    Bottle &bDominantEye=options.addList();
    bDominantEye.addString("dominantEye");
    bDominantEye.addString(dominantEye.c_str());

    mutex.post();
}


/************************************************************************/
void Localizer::setPidOptions(const Bottle &options)
{
    mutex.wait();

    pid->setOptions(options);
    Bottle &opt=const_cast<Bottle&>(options);
    if (opt.check("dominantEye"))
    {
        string domEye=opt.find("dominantEye").asString().c_str();
        if ((domEye=="left") || (domEye=="right"))
            dominantEye=domEye;
    }

    mutex.post();
}


/************************************************************************/
Vector Localizer::getAbsAngles(const Vector &x)
{
    Vector fp=x;
    fp.push_back(1.0);  // impose homogeneous coordinates

    // get fp wrt head-centered system
    Vector fph=invEyeCAbsFrame*fp;
    fph.pop_back();

    Vector ang(3);
    ang[0]=atan2(fph[0],fph[2]);
    ang[1]=-atan2(fph[1],fabs(fph[2]));
    ang[2]=2.0*atan2(eyesHalfBaseline,norm(fph));

    return ang;
}


/************************************************************************/
Vector Localizer::get3DPoint(const string &type, const Vector &ang)
{
    double azi=ang[0];
    double ele=ang[1];
    double ver=ang[2];

    //Vector q(8,0.0);
    Vector q(5,0.0);
    if (type=="rel")
    {
        Vector torso=commData->get_torso();
        Vector head=commData->get_q();

        q[0]=torso[0];
        /*q[1]=torso[1];
        q[2]=torso[2];
        q[3]=head[0];
        q[4]=head[1];
        q[5]=head[2];
        q[6]=head[3];
        q[7]=head[4];

        ver+=head[5];*/
        q[1]=head[0];
        q[2]=head[1];
        q[3]=head[2];
        q[4]=head[3];

        ver+=head[4];
    }
    
    // impose vergence != 0.0
    if (ver<commData->get_minAllowedVergence())
        ver=commData->get_minAllowedVergence();

    mutex.wait();

    //q[7]+=ver/2.0;
    // temporary fix while the firmware is updated
    q[4]=(q[4]+ver)/2.0;
    //q[4]+=ver/2.0;
    eyeL->setAng(q);

    //q[7]-=ver;
    // temporary fix while the firmware is updated
    q[4]=(q[4]-ver)/2.0;
    //q[4]-=ver;
    eyeR->setAng(q);

    Vector fp(4);
    fp[3]=1.0;  // impose homogeneous coordinates

    // compute new fp due to changed vergence
    computeFixationPointOnly(*(eyeL->asChain()),*(eyeR->asChain()),fp);

    mutex.post();

    // compute rotational matrix to
    // account for elevation and azimuth
    Vector x(4), y(4);
    x[0]=1.0;    y[0]=0.0;
    x[1]=0.0;    y[1]=1.0;
    x[2]=0.0;    y[2]=0.0;
    x[3]=ele;    y[3]=azi;   
    Matrix R=axis2dcm(y)*axis2dcm(x);

    Vector fph, xd;
    if (type=="rel")
    {
        Matrix frame=commData->get_fpFrame();
        fph=SE3inv(frame)*fp;       // get fp wrt relative head-centered frame
        xd=frame*(R*fph);           // apply rotation and retrieve fp wrt root frame
    }
    else
    {
        fph=invEyeCAbsFrame*fp;     // get fp wrt absolute head-centered frame
        xd=eyeCAbsFrame*(R*fph);    // apply rotation and retrieve fp wrt root frame
    }

    return xd.subVector(0,2);
}


/************************************************************************/
bool Localizer::projectPoint(const string &type, const Vector &x, Vector &px)
{
    if (x.length()<3)
    {
        fprintf(stdout,"Not enough values given for the point!\n");
        return false;
    }

    bool isLeft=(type=="left");

    Matrix  *Prj=(isLeft?PrjL:PrjR);
    vizzyEye *eye=(isLeft?eyeL:eyeR);

    if (Prj)
    {
        Vector torso=commData->get_torso();
        Vector head=commData->get_q();

        //Vector q(8);
        Vector q(5);
        q[0]=torso[0];
        /*q[1]=torso[1];
        q[2]=torso[2];
        q[3]=head[0];
        q[4]=head[1];
        q[5]=head[2];
        q[6]=head[3];*/
        q[1]=head[0];
        q[2]=head[1];
        q[3]=head[2];

        if (isLeft)
            //q[7]=head[4]+head[5]/2.0;
            // temporary fix while the firmware is updated
	    q[4]=(head[3]+head[4])/2.0;
            //q[4]=head[3]+head[4]/2.0;
        else
            //q[7]=head[4]-head[5]/2.0;
	    // temporary fix while the firmware is updated
	    q[4]=(head[3]-head[4])/2.0;
            //q[4]=head[3]-head[4]/2.0;
        
        Vector xo=x;
        if (xo.length()<4)
            xo.push_back(1.0);  // impose homogeneous coordinates

        // find position wrt the camera frame
        mutex.wait();
        Vector xe=SE3inv(eye->getH(q))*xo;
        mutex.post();

        // find the 2D projection
        px=*Prj*xe;
        px=px/px[2];
        px=px.subVector(0,1);

        return true;
    }
    else
    {
        fprintf(stdout,"Unspecified projection matrix for %s camera!\n",type.c_str());
        return false;
    }
}


/************************************************************************/
bool Localizer::projectPoint(const string &type, const double u, const double v,
                             const double z, Vector &x)
{
    bool isLeft=(type=="left");

    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    vizzyEye *eye=(isLeft?eyeL:eyeR);

    if (invPrj)
    {
        Vector torso=commData->get_torso();
        Vector head=commData->get_q();

        //Vector q(8);
        Vector q(5);
         q[0]=torso[0];
         /*q[1]=torso[1];
         q[2]=torso[2];
         q[3]=head[0];
         q[4]=head[1];
         q[5]=head[2];
         q[6]=head[3];*/
         q[1]=head[0];
         q[2]=head[1];
         q[3]=head[2];

        if (isLeft)
        	//q[7]=head[4]+head[5]/2.0;
		// temporary fix while the firmware is updated
        	q[4]=(head[3]+head[4])/2.0;
        	//q[4]=head[3]+head[4]/2.0;
        else
        	//q[7]=head[4]-head[5]/2.0;
		// temporary fix while the firmware is updated
		q[4]=(head[3]-head[4])/2.0;
        	//q[4]=head[3]-head[4]/2.0;

        Vector p(3);
        p[0]=z*u;
        p[1]=z*v;
        p[2]=z;

        // find the 3D position from the 2D projection,
        // knowing the coordinate z in the camera frame
        Vector xe=*invPrj*p;
        xe[3]=1.0;  // impose homogeneous coordinates

        // find position wrt the root frame
        mutex.wait();
        x=(eye->getH(q)*xe).subVector(0,2);
        mutex.post();

        return true;
    }
    else
    {
        fprintf(stdout,"Unspecified projection matrix for %s camera!\n",type.c_str());
        return false;
    }
}


/************************************************************************/
bool Localizer::projectPoint(const string &type, const double u, const double v,
                             const Vector &plane, Vector &x)
{
    if (plane.length()<4)
    {
        fprintf(stdout,"Not enough values given for the projection plane!\n");
        return false;
    }

    bool isLeft=(type=="left");
    vizzyEye *eye=(isLeft?eyeL:eyeR);

    if (projectPoint(type,u,v,1.0,x))
    {
        // pick up a point belonging to the plane
        Vector p0(3,0.0);
        if (plane[0]!=0.0)
            p0[0]=-plane[3]/plane[0];
        else if (plane[1]!=0.0)
            p0[1]=-plane[3]/plane[1];
        else if (plane[2]!=0.0)
            p0[2]=-plane[3]/plane[2];
        else
        {
            fprintf(stdout,"Error while specifying projection plane!\n");
            return false;
        }

        // take a vector orthogonal to the plane
        Vector n(3);
        n[0]=plane[0];
        n[1]=plane[1];
        n[2]=plane[2];

        mutex.wait();
        Vector e=eye->EndEffPose().subVector(0,2);
        mutex.post();

        // compute the projection
        Vector v=x-e;
        x=e+(dot(p0-e,n)/dot(v,n))*v;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool Localizer::triangulatePoint(const Vector &pxl, const Vector &pxr, Vector &x)
{
    if ((pxl.length()<2) || (pxr.length()<2))
    {
        fprintf(stdout,"Not enough values given for the pixels!\n");
        return false;
    }

    if (PrjL && PrjR)
    {
        Vector torso=commData->get_torso();
        Vector head=commData->get_q();

        /*Vector qL(8);
        qL[0]=torso[0];
        qL[1]=torso[1];
        qL[2]=torso[2];
        qL[3]=head[0];
        qL[4]=head[1];
        qL[5]=head[2];
        qL[6]=head[3];
        qL[7]=head[4]+head[5]/2.0;*/

        Vector qL(5);
        qL[0]=torso[0];
        qL[1]=head[0];
        qL[2]=head[1];
        qL[3]=head[2];
        //qL[4]=head[3]+head[4]/2.0;
	qL[4]=(head[3]+head[4])/2.0;


        Vector qR=qL;
        //qR[7]-=head[5];
        //qR[4]-=head[4];
	qL[4]=(head[3]-head[4])/2.0;

        mutex.wait();
        Matrix HL=SE3inv(eyeL->getH(qL));
        Matrix HR=SE3inv(eyeR->getH(qR));
        mutex.post();

        Matrix tmp=zeros(3,4); tmp(2,2)=1.0;
        tmp(0,2)=pxl[0]; tmp(1,2)=pxl[1];
        Matrix AL=(*PrjL-tmp)*HL;

        tmp(0,2)=pxr[0]; tmp(1,2)=pxr[1];
        Matrix AR=(*PrjR-tmp)*HR;

        Matrix A(4,3);
        Vector b(4);
        for (int i=0; i<2; i++)
        {
            b[i]=-AL(i,3);
            b[i+2]=-AR(i,3);

            for (int j=0; j<3; j++)
            {
                A(i,j)=AL(i,j);
                A(i+2,j)=AR(i,j);
            }
        }

        // solve the least-squares problem
        x=pinv(A)*b;

        return true;
    }
    else
    {
        fprintf(stdout,"Unspecified projection matrix for at least one camera!\n");
        return false;
    }
}


/************************************************************************/
void Localizer::handleMonocularInput()
{
    if (Bottle *mono=port_mono.read(false))
    {
        if (mono->size()>=4)
        {
            string type=mono->get(0).asString().c_str();
            double u=mono->get(1).asDouble();
            double v=mono->get(2).asDouble();
            double z;

            if (mono->get(3).isDouble())
                z=mono->get(3).asDouble();
            else if (mono->get(3).asString()=="ver")
            {
                double ver=CTRL_DEG2RAD*mono->get(4).asDouble();
                double tg=tan(ver/2.0);
                z=eyesHalfBaseline*sqrt(1.0+1.0/(tg*tg));
            }
            else
            {
                fprintf(stdout,"Got wrong mono information!\n");
                return;
            }

            Vector fp;
            if (projectPoint(type,u,v,z,fp))
            {
                if (port_xd!=NULL)
                    port_xd->set_xd(fp);
                else
                    fprintf(stdout,"Internal error occured!\n");
            }
        }
        else
            fprintf(stdout,"Got wrong mono information!\n");
    }
}


/************************************************************************/
void Localizer::handleStereoInput()
{
    if (Bottle *stereo=port_stereo.read(false))
    {
        if ((PrjL!=NULL) || (PrjR!=NULL))
        {
            if (stereo->size()>=4)
            {
                double ul=stereo->get(0).asDouble();
                double vl=stereo->get(1).asDouble();
                double ur=stereo->get(2).asDouble();
                double vr=stereo->get(3).asDouble();

                Vector ref(1), fb(1), fp;
                double u, v;

                ref=0.0;
                if (dominantEye=="left")
                {
                    u=ul;
                    v=vl;
                    fb=cxr-ur;
                    // by inverting the sign of the error (e=ref-fb=-fb)
                    // we can keep gains always positive
                }
                else
                {                    
                    u=ur;
                    v=vr;
                    fb=ul-cxl;
                }
                
                mutex.wait();
                Vector z=pid->compute(ref,fb);
                mutex.post();

                if (projectPoint(dominantEye,u,v,z[0],fp))
                {
                    if (port_xd!=NULL)
                        port_xd->set_xd(fp);
                    else
                        fprintf(stdout,"Internal error occured!\n");
                }
            }
            else
                fprintf(stdout,"Got wrong stereo information!\n");
        }
        else
            fprintf(stdout,"Unspecified projection matrix!\n");
    }
}


/************************************************************************/
void Localizer::handleAnglesInput()
{
    if (Bottle *angles=port_anglesIn.read(false))
    {
        if (angles->size()>=4)
        {
            Vector ang(3);
        
            string type=angles->get(0).asString().c_str();
            ang[0]=CTRL_DEG2RAD*angles->get(1).asDouble();
            ang[1]=CTRL_DEG2RAD*angles->get(2).asDouble();
            ang[2]=CTRL_DEG2RAD*angles->get(3).asDouble();

            Vector xd=get3DPoint(type,ang);
        
            if (port_xd!=NULL)
                port_xd->set_xd(xd);
            else
                fprintf(stdout,"Internal error occured!\n");
        }
        else
            fprintf(stdout,"Got wrong angles information!\n");
    }
}


/************************************************************************/
void Localizer::handleAnglesOutput()
{
    double x_stamp;
    Vector x=commData->get_x(x_stamp);
    txInfo_ang.update(x_stamp);

    if (port_anglesOut.getOutputCount()>0)
    {
        port_anglesOut.prepare()=CTRL_RAD2DEG*getAbsAngles(x);
        port_anglesOut.setEnvelope(txInfo_ang);
        port_anglesOut.write();
	/*Bottle angles_message = Bottle();
	Bottle& list_1 = angles_message.addList();
	list_1.add(txInfo_ang.getCount());
	Bottle& list_2 = angles_message.addList();
	Vector angles_converted(CTRL_RAD2DEG*getAbsAngles(x));
	for (int my_i=0;my_i<x.size();my_i++){
	    list_2.add(angles_converted[my_i]);
	}
        ros_port_anglesOut.write(angles_message);
	Time::delay(0.1);
	ros_port_anglesOut.write(angles_message);
	ros_port_anglesOut.write(angles_message);*/

    }
}


/************************************************************************/
void Localizer::run()
{
    handleMonocularInput();
    handleStereoInput();
    handleAnglesInput();
    handleAnglesOutput();
}


/************************************************************************/
void Localizer::threadRelease()
{
    port_mono.interrupt();
    port_stereo.interrupt();
    port_anglesIn.interrupt();
    port_anglesOut.interrupt();

    port_mono.close();
    port_stereo.close();
    port_anglesIn.close();
    port_anglesOut.close();

    delete eyeL;
    delete eyeR;
    delete pid;

    if (PrjL!=NULL)
    {
        delete PrjL;
        delete invPrjL;
    }

    if (PrjR!=NULL)
    {
        delete PrjR;
        delete invPrjR;
    }
}



