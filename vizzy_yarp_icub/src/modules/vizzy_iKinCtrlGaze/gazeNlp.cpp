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

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <vizzy/gazeNlp.h>
#include <vizzy/utils.h>


/************************************************************************/
bool computeFixationPointData(iKinChain &eyeL, iKinChain &eyeR, Vector &fp, Matrix &J)
{
    Vector dfp1(4), dfp2(4);
    Vector dfpL1(4),dfpL2(4);
    Vector dfpR1(4),dfpR2(4);

    Matrix HL=eyeL.getH();
    Matrix HR=eyeR.getH();

    HL(3,3)=HR(3,3)=0.0;

    double qty1=dot(HR,2,HL,2);
    Matrix H1=HL-HR;
    Matrix H2L=HL-qty1*HR;
    Matrix H2R=qty1*HL-HR;
    Matrix H3(4,4); H3(3,2)=0.0;
    double qty2L=dot(H2L,2,H1,3);
    double qty2R=dot(H2R,2,H1,3);
    double qty3=qty1*qty1-1.0;

    if (fabs(qty3)<ALMOST_ZERO)
        return false;

    double tL=qty2L/qty3;
    double tR=qty2R/qty3;

    Matrix GeoJacobP_L=eyeL.GeoJacobian();
    Matrix GeoJacobP_R=eyeR.GeoJacobian();
    Matrix AnaJacobZ_L=eyeL.AnaJacobian(2);
    Matrix AnaJacobZ_R=eyeR.AnaJacobian(2);

    // Left part
    {
        double dqty1, dqty1L, dqty1R;
        double dqty2, dqty2L, dqty2R;
        int    j;

        Vector Hz=HL.getCol(2);
        Matrix M=GeoJacobP_L.submatrix(0,3,0,1)+tL*AnaJacobZ_L.submatrix(0,3,0,1);

        // derivative wrt eye tilt
        j=0;
        dqty1=dot(AnaJacobZ_R,j,HL,2)+dot(HR,2,AnaJacobZ_L,j);
        for (int i=0; i<3; i++)
            H3(i,2)=AnaJacobZ_L(i,j)-dqty1*HR(i,2)-qty1*AnaJacobZ_R(i,j);
        dqty2=dot(H3,2,H1,3)+dot(H2L,2,GeoJacobP_L-GeoJacobP_R,j);
        dfp1=M.getCol(j)+Hz*((dqty2-2.0*qty1*qty2L*dqty1/qty3)/qty3);

        // derivative wrt pan left eye
        j=1;
        dqty1L=dot(HR,2,AnaJacobZ_L,j);
        for (int i=0; i<3; i++)
            H3(i,2)=AnaJacobZ_L(i,j)-dqty1*HR(i,2);
        dqty2L=dot(H3,2,H1,3)+dot(H2L,2,GeoJacobP_L,j);
        dfpL1=M.getCol(j)+Hz*((dqty2L-2.0*qty1*qty2L*dqty1L/qty3)/qty3);

        // derivative wrt pan right eye
        dqty1R=dot(AnaJacobZ_R,j,HL,2);
        for (int i=0; i<3; i++)
            H3(i,2)=-dqty1*HR(i,2)-qty1*AnaJacobZ_R(i,j);
        dqty2R=dot(H3,2,H1,3)+dot(H2L,2,-1.0*GeoJacobP_R,j);
        dfpR1=Hz*((dqty2R-2.0*qty1*qty2L*dqty1R/qty3)/qty3);
    }

    // Right part
    {
        double dqty1, dqty1L, dqty1R;
        double dqty2, dqty2L, dqty2R;
        int    j;

        Vector Hz=HR.getCol(2);
        Matrix M=GeoJacobP_R.submatrix(0,3,0,1)+tR*AnaJacobZ_R.submatrix(0,3,0,1);

        // derivative wrt eye tilt
        j=0;
        dqty1=dot(AnaJacobZ_R,j,HL,2)+dot(HR,2,AnaJacobZ_L,j);
        for (int i=0; i<3; i++)
            H3(i,2)=qty1*AnaJacobZ_L(i,j)+dqty1*HL(i,2)-AnaJacobZ_R(i,j);
        dqty2=dot(H3,2,H1,3)+dot(H2R,2,GeoJacobP_L-GeoJacobP_R,j);
        dfp2=M.getCol(j)+Hz*((dqty2-2.0*qty1*qty2R*dqty1/qty3)/qty3);

        // derivative wrt pan left eye
        j=1;
        dqty1L=dot(HR,2,AnaJacobZ_L,j);
        for (int i=0; i<3; i++)
            H3(i,2)=qty1*AnaJacobZ_L(i,j)+dqty1L*HL(i,2);
        dqty2L=dot(H3,2,H1,3)+dot(H2R,2,GeoJacobP_L,j);
        dfpL2=Hz*((dqty2L-2.0*qty1*qty2R*dqty1L/qty3)/qty3);

        // derivative wrt pan right eye
        dqty1R=dot(AnaJacobZ_R,j,HL,2);
        for (int i=0; i<3; i++)
            H3(i,2)=dqty1R*HL(i,2)-AnaJacobZ_R(i,j);
        dqty2R=dot(H3,2,H1,3)+dot(H2R,2,-1.0*GeoJacobP_R,j);
        dfpR2=M.getCol(j)+Hz*((dqty2R-2.0*qty1*qty2R*dqty1R/qty3)/qty3);
    }

    for (int i=0; i<3; i++)
    {
        // fixation point position
        fp[i]=0.5*(HL(i,3)+tL*HL(i,2)+HR(i,3)+tR*HR(i,2));

        // Jacobian
        // r=p-v/2, l=p+v/2;
        // dfp/dp=dfp/dl*dl/dp + dfp/dr*dr/dp = dfp/dl + dfp/dr;
        // dfp/dv=dfp/dl*dl/dv + dfp/dr*dr/dv = (dfp/dl - dfp/dr)/2;
        J(i,0)=0.50*(dfp1[i]           + dfp2[i]);              // tilt
        J(i,1)=0.50*(dfpL1[i]+dfpR1[i] + dfpL2[i]+dfpR2[i]);    // pan
        J(i,2)=0.25*(dfpL1[i]-dfpR1[i] + dfpL2[i]-dfpR2[i]);    // vergence
    }

    return true;
}


/************************************************************************/
bool computeFixationPointOnly(iKinChain &eyeL, iKinChain &eyeR, Vector &fp)
{
    Vector dfp1(4), dfp2(4);
    Vector dfpL1(4),dfpL2(4);
    Vector dfpR1(4),dfpR2(4);

    Matrix HL=eyeL.getH();
    Matrix HR=eyeR.getH();
    HL(3,3)=HR(3,3)=0.0;

    double qty1=dot(HR,2,HL,2);
    Matrix H1=HL-HR;
    Matrix H2L=HL-qty1*HR;
    Matrix H2R=qty1*HL-HR;
    Matrix H3(4,4); H3(3,2)=0.0;
    double qty2L=dot(H2L,2,H1,3);
    double qty2R=dot(H2R,2,H1,3);
    double qty3=qty1*qty1-1.0;

    if (fabs(qty3)<ALMOST_ZERO)
        return false;

    double tL=qty2L/qty3;
    double tR=qty2R/qty3;

    for (int i=0; i<3; i++)
        fp[i]=0.5*(HL(i,3)+tL*HL(i,2)+HR(i,3)+tR*HR(i,2));

    return true;
}


/************************************************************************/
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

#if 0
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
#else
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
#endif
}


/************************************************************************/
/*void HeadCenter_NLP::computeQuantities(const Ipopt::Number *x)
{
    Vector new_q(dim);
    for (Ipopt::Index i=0; i<(int)dim; i++)
        new_q[i]=x[i];

    if (!(q==new_q) || firstGo)
    {
        firstGo=false;
        q=new_q;

        q=chain.setAng(q);
        Hxd=chain.getH();
        Hxd(0,3)-=xd[0];
        Hxd(1,3)-=xd[1];
        Hxd(2,3)-=xd[2];
        Hxd(3,3)=0.0;
        mod=norm(Hxd,3);
        cosAng=dot(Hxd,2,Hxd,3)/mod;

        GeoJacobP=chain.GeoJacobian();
        AnaJacobZ=chain.AnaJacobian(2);
    }
}*/


/************************************************************************/
/*bool HeadCenter_NLP::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                                  Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n=dim;
    m=nnz_jac_g=nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    
    return true;
}*/


/************************************************************************/
/*bool HeadCenter_NLP::get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                     Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u)
{
    for (Ipopt::Index i=0; i<n; i++)
    {
        x_l[i]=chain(i).getMin();
        x_u[i]=chain(i).getMax();
    }

    return true;
}*/


/************************************************************************/
/*bool HeadCenter_NLP::eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Number& obj_value)
{
    computeQuantities(x);
    obj_value=cosAng;
    return true;
}*/


/************************************************************************/
/*bool HeadCenter_NLP::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x,
                                 bool new_x, Ipopt::Number* grad_f)
{
    computeQuantities(x);
    for (Ipopt::Index i=0; i<n; i++)
    {
        grad_f[i]=(dot(AnaJacobZ,i,Hxd,3)+dot(Hxd,2,GeoJacobP,i))/mod
                  -(cosAng*dot(Hxd,3,GeoJacobP,i))/(mod*mod);
    }

    return true;
}*/

/*********************************************************************************************************/

class HeadCenter_NLP : public Ipopt::TNLP
{
private:
    // Copy constructor: not implemented.
    HeadCenter_NLP(const HeadCenter_NLP&);
    // Assignment operator: not implemented.
    HeadCenter_NLP &operator=(const HeadCenter_NLP&);

protected:

    iKinChain dummyChain;
    Vector    dummyVector;

    Matrix Hxd;
    Matrix GeoJacobP;
    Matrix AnaJacobZ;

    double mod;
    double cosAng;

    void computeQuantities(const Ipopt::Number *x) {
      Vector new_q(dim);
      for (Ipopt::Index i=0; i<(int)dim; i++)
          new_q[i]=x[i];

      if (!(q==new_q) || firstGo)
      {
          firstGo=false;
          q=new_q;

          q=chain.setAng(q);
          Hxd=chain.getH();
          Hxd(0,3)-=xd[0];
          Hxd(1,3)-=xd[1];
          Hxd(2,3)-=xd[2];
          Hxd(3,3)=0.0;
          mod=norm(Hxd,3);
          cosAng=dot(Hxd,2,Hxd,3)/mod;

          GeoJacobP=chain.GeoJacobian();
          AnaJacobZ=chain.AnaJacobian(2);
      }
    }

    // this attributes were from the old class iKin_NLP ##################################################################
    iKinChain &chain;


    unsigned int dim;
    unsigned int dim_2nd;
    unsigned int ctrlPose;

    yarp::sig::Vector  qd;
    yarp::sig::Vector  q;
    yarp::sig::Vector  q0;
    yarp::sig::Vector &xd;
    double weight2ndTask;
    iKinChain &chain2ndTask;
    yarp::sig::Vector &xd_2nd;
    yarp::sig::Vector &w_2nd;
    double weight3rdTask;
    yarp::sig::Vector &qd_3rd;
    yarp::sig::Vector &w_3rd;
    iKinLinIneqConstr &LIC;
    bool   *exhalt;

    yarp::sig::Vector  e_zero;
    yarp::sig::Vector  e_xyz;
    yarp::sig::Vector  e_ang;
    yarp::sig::Vector  e_2nd;
    yarp::sig::Vector  e_3rd;

    yarp::sig::Matrix  J_zero;
    yarp::sig::Matrix  J_xyz;
    yarp::sig::Matrix  J_ang;
    yarp::sig::Matrix  J_2nd;

    yarp::sig::Vector *e_1st;
    yarp::sig::Matrix *J_1st;

    yarp::sig::Vector linC;

    Ipopt::Number __obj_scaling;
    Ipopt::Number __x_scaling;
    Ipopt::Number __g_scaling;

    Ipopt::Number lowerBoundInf;
    Ipopt::Number upperBoundInf;

    Ipopt::Number translationalTol;

    iKinIterateCallback *callback;
    bool firstGo;

    // end of "this attributes were from the old class iKin_NLP"  ##################################################################

public:

    HeadCenter_NLP(iKinChain &c, unsigned int _ctrlPose, const yarp::sig::Vector &_q0, yarp::sig::Vector &_xd,
                   double _weight2ndTask, iKinChain &_chain2ndTask, yarp::sig::Vector &_xd_2nd, yarp::sig::Vector &_w_2nd,
                   double _weight3rdTask, yarp::sig::Vector &_qd_3rd, yarp::sig::Vector &_w_3rd,
                   iKinLinIneqConstr &_LIC, bool *_exhalt) :
                   chain(c), q0(_q0), xd(_xd),
                   chain2ndTask(_chain2ndTask),   xd_2nd(_xd_2nd), w_2nd(_w_2nd),
                   weight3rdTask(_weight3rdTask),qd_3rd(_qd_3rd), w_3rd(_w_3rd),
                   LIC(_LIC), 
                   exhalt(_exhalt)
  {
      dim=chain.getDOF();
      dim_2nd=chain2ndTask.getDOF();

      ctrlPose=_ctrlPose;

      if (ctrlPose>IKINCTRL_POSE_ANG)
          ctrlPose=IKINCTRL_POSE_ANG;

      weight2ndTask=dim_2nd>0 ? _weight2ndTask : 0.0;

      qd.resize(dim);

      unsigned int n=q0.length();
      n=n>dim ? dim : n;

      unsigned int i;
      for (i=0; i<n; i++)
          qd[i]=q0[i];

      for (; i<dim; i++)
          qd[i]=0.0;

      q=qd;

      e_zero.resize(3,0.0);
      e_xyz.resize(3,0.0);
      e_ang.resize(3,0.0);
      e_2nd.resize(3,0.0);
      e_3rd.resize(dim,0.0);

      J_zero.resize(3,dim); J_zero.zero();
      J_xyz.resize(3,dim);  J_xyz.zero();
      J_ang.resize(3,dim);  J_ang.zero();
      J_2nd.resize(3,dim);  J_2nd.zero();

      if (ctrlPose==IKINCTRL_POSE_FULL)
      {
          e_1st=&e_ang;
          J_1st=&J_ang;
      }
      else
      {
          e_1st=&e_zero;
          J_1st=&J_zero;
      }

      firstGo=true;

      __obj_scaling=1.0;
      __x_scaling  =1.0;
      __g_scaling  =1.0;

      lowerBoundInf=IKINIPOPT_DEFAULT_LWBOUNDINF;
      upperBoundInf=IKINIPOPT_DEFAULT_UPBOUNDINF;
      translationalTol=IKINIPOPT_DEFAULT_TRANSTOL;

      callback=NULL;
  }
    
    /** default constructor */
    HeadCenter_NLP(iKinChain &c, const Vector &_q0, Vector &_xd,
                   iKinLinIneqConstr &_LIC, bool *_exhalt=NULL) :
                   HeadCenter_NLP(c,IKINCTRL_POSE_XYZ,_q0,_xd,
                            0.0,dummyChain,dummyVector,dummyVector,
                            0.0,dummyVector,dummyVector,
                            _LIC,_exhalt) { }

    // functions from old iKin_NLP  ##################################################################
    
    /** returns the solution */
    yarp::sig::Vector get_qd() { return qd; }

    /** sets callback */
    void set_callback(iKinIterateCallback *_callback) { callback=_callback; }

    /** sets scaling factors */
    void set_scaling(Ipopt::Number _obj_scaling, Ipopt::Number _x_scaling, Ipopt::Number _g_scaling)
    {
        __obj_scaling=_obj_scaling;
        __x_scaling  =_x_scaling;
        __g_scaling  =_g_scaling;
    }

    /** sets scaling factors */
    void set_bound_inf(Ipopt::Number lower, Ipopt::Number upper)
    {
        lowerBoundInf=lower;
        upperBoundInf=upper;
    }

    /** sets translational tolerance */
    void set_translational_tol(Ipopt::Number tol) { translationalTol=tol; }

    /** default destructor */
    virtual ~HeadCenter_NLP() { }
    
    // end functions from old iKin_NLP  ##################################################################

    /** Method to return some info about the nlp */
    bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                      Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) {
      n=dim;
      m=nnz_jac_g=nnz_h_lag=0;
      index_style=TNLP::C_STYLE;
      
      return true;                 
    }
    
    /** Method to return the bounds for my problem */
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                         Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) {
    
      for (Ipopt::Index i=0; i<n; i++)
      {
          x_l[i]=chain(i).getMin();
          x_u[i]=chain(i).getMax();
      }

      return true;
                        
    }                     
    
    // functions from old iKin_NLP  ##################################################################
    
    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda,
                                    Ipopt::Number* lambda) {
    
      for (Ipopt::Index i=0; i<n; i++)
        x[i]=q0[i];

      return true;
                                   
    }
                                    
    // end functions from old iKin_NLP   ##################################################################
    
    /** Method to return the objective value */
    bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) {
    
      computeQuantities(x);
      obj_value=cosAng;
      return true;
    }
    
    /** Method to return the gradient of the objective */
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) {
    
      computeQuantities(x);
      for (Ipopt::Index i=0; i<n; i++)
      {
          grad_f[i]=(dot(AnaJacobZ,i,Hxd,3)+dot(Hxd,2,GeoJacobP,i))/mod
                    -(cosAng*dot(Hxd,3,GeoJacobP,i))/(mod*mod);
      }

      return true;
    
    }
    
    /** Method to return the constraint residuals */
    bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) { return true; }
    
    /** Method to return:
    *   1) The structure of the jacobian (if "values" is NULL)
    *   2) The values of the jacobian (if "values" is not NULL)
    */
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,Ipopt::Index m, Ipopt::Index nele_jac,
                            Ipopt::Index* iRow, Ipopt::Index *jCol, Ipopt::Number* values) { return true; }
                            
    // functions from old iKin_NLP   ##################################################################                        
    
    /** Method to return:
    *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
    *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
    */
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values) {
                        
      if (!values)
      {
          Ipopt::Index idx=0;
      
          for (Ipopt::Index row=0; row<n; row++)
          {
              for (Ipopt::Index col=0; col<=row; col++)
              {
                  iRow[idx]=row;
                  jCol[idx]=col;
                  idx++;
              }
          }
      }
      else
      {
          // Given the task: min f(q)=1/2*||xd-F(q)||^2
          // the Hessian Hij is: <dF/dqi,dF/dqj> - <d2F/dqidqj,e>

          computeQuantities(x);

          chain.prepareForHessian();

          if (weight2ndTask!=0.0)
              chain2ndTask.prepareForHessian();

          Ipopt::Index idx=0;

          for (Ipopt::Index row=0; row<n; row++)
          {
              for (Ipopt::Index col=0; col<=row; col++)
              {
                  // warning: row and col are swapped due to asymmetry
                  // of orientation part within the hessian 
                  yarp::sig::Vector h=chain.fastHessian_ij(col,row);
                  yarp::sig::Vector h_xyz(3), h_ang(3), h_zero(3,0.0);
                  h_xyz[0]=h[0];
                  h_xyz[1]=h[1];
                  h_xyz[2]=h[2];
                  h_ang[0]=h[3];
                  h_ang[1]=h[4];
                  h_ang[2]=h[5];
              
                  yarp::sig::Vector *h_1st;
                  if (ctrlPose==IKINCTRL_POSE_FULL)
                      h_1st=&h_ang;
                  else
                      h_1st=&h_zero;
              
                  values[idx]=obj_factor*(dot(*J_1st,row,*J_1st,col)-dot(*h_1st,*e_1st));            
                  values[idx]+=lambda[0]*(dot(J_xyz,row,J_xyz,col)-dot(h_xyz,e_xyz));
              
                  if ((weight2ndTask!=0.0) && (row<(int)dim_2nd) && (col<(int)dim_2nd))
                  {    
                      // warning: row and col are swapped due to asymmetry
                      // of orientation part within the hessian 
                      yarp::sig::Vector h2=chain2ndTask.fastHessian_ij(col,row);
                      yarp::sig::Vector h_2nd(3);
                      h_2nd[0]=(w_2nd[0]*w_2nd[0])*h2[0];
                      h_2nd[1]=(w_2nd[1]*w_2nd[1])*h2[1];
                      h_2nd[2]=(w_2nd[2]*w_2nd[2])*h2[2];
              
                      values[idx]+=obj_factor*weight2ndTask*(dot(J_2nd,row,J_2nd,col)-dot(h_2nd,e_2nd));
                  }
              
                  idx++;
              }
          }
      }
      
      return true;                 
    }

    /** overload this method to return scaling parameters. This is
    *  only called if the options are set to retrieve user scaling.
    *  There, use_x_scaling (or use_g_scaling) should get set to true
    *  only if the variables (or constraints) are to be scaled.  This
    *  method should return true only if the scaling parameters could
    *  be provided.
    */
    virtual bool get_scaling_parameters(Ipopt::Number& obj_scaling,
                                        bool& use_x_scaling, Ipopt::Index n, Ipopt::Number* x_scaling,
                                        bool& use_g_scaling, Ipopt::Index m, Ipopt::Number* g_scaling) {
      obj_scaling=__obj_scaling;

      for (Ipopt::Index i=0; i<n; i++)
          x_scaling[i]=__x_scaling;

      for (Ipopt::Index j=0; j<m; j++)
          g_scaling[j]=__g_scaling;

      use_x_scaling=use_g_scaling=true;

      return true;                                   
                                        
    }

    /** This method is called once per iteration, after the iteration
     *  summary output has been printed.
     */
    virtual bool intermediate_callback(Ipopt::AlgorithmMode mode, Ipopt::Index iter, Ipopt::Number obj_value,
                                       Ipopt::Number inf_pr, Ipopt::Number inf_du, Ipopt::Number mu, Ipopt::Number d_norm,
                                       Ipopt::Number regularization_size, Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                                       Ipopt::Index ls_trials, const Ipopt::IpoptData* ip_data,
                                       Ipopt::IpoptCalculatedQuantities* ip_cq) {
      if (callback!=NULL)
        callback->exec(xd,q);

      if (exhalt!=NULL)
          return !(*exhalt);
      else
          return true;                                  
                                       
    }
    
    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x,
                                   const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m,
                                   const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq) {
                                   
      for (Ipopt::Index i=0; i<n; i++)
        qd[i]=x[i];

      qd=chain.setAng(qd);                              
                                   
    }
                                   
   // end functions from old iKin_NLP   ##################################################################                                
};

/*********************************************************************************************************/

// Solve through IPOPT the nonlinear problem 
/*class GazeIpOptMin : public iKinIpOptMin
{
private:
    GazeIpOptMin();
    GazeIpOptMin(const GazeIpOptMin&);
    GazeIpOptMin &operator=(const GazeIpOptMin&);

public:
    GazeIpOptMin(iKinChain &_chain, const double tol, const int max_iter=IKINCTRL_DISABLED,
                 const unsigned int verbose=0) :
                 iKinIpOptMin(_chain,IKINCTRL_POSE_XYZ,tol,max_iter,verbose,false) { }

    void   set_ctrlPose(const unsigned int _ctrlPose) { }
    void   setHessianOpt(const bool useHessian)       { }   // Hessian not implemented
    Vector solve(const Vector &q0, Vector &xd, const Vector &gDir, Ipopt::ApplicationReturnStatus *exit_code=NULL,
                 bool *exhalt=NULL, iKinIterateCallback *iterate=NULL);
};*/

/************************************************************************/
Vector GazeIpOptMin::solve(const Vector &q0, Vector &xd, const Vector &gDir,
                           bool *exhalt, iKinIterateCallback *iterate)
{
    Ipopt::SmartPtr<HeadCenter_NLP> nlp;
    nlp=new HeadCenter_NLP(chain,q0,xd,*pLIC,exhalt);

    nlp->set_scaling(obj_scaling,x_scaling,g_scaling);
    nlp->set_bound_inf(lowerBoundInf,upperBoundInf);
    //nlp->set_translational_tol(translationalTol);
    nlp->set_translational_tol(IKINIPOPT_DEFAULT_TRANSTOL);
    nlp->set_callback(iterate);
    
    /*Ipopt::ApplicationReturnStatus status=optimize(GetRawPtr(nlp));

    if (exit_code!=NULL)
        *exit_code=status;*/
        
    static_cast<Ipopt::IpoptApplication*>(App)->OptimizeTNLP(GetRawPtr(nlp));

    return nlp->get_qd();
}

