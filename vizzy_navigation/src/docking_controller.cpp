/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <docking_controller.hpp>
#include <iostream>
#include <math.h>

namespace docking_ctrl{

DockingController::DockingController(float k_ro, float k_alpha, float k_beta, Pose2D goal, Pose2D robot_pose) : k_ro_(k_ro), 
        k_alpha_(k_alpha), k_beta_(k_beta), robot_pose_(robot_pose), goal_(goal), 
        lin_vel_sat_(2.0), ang_vel_sat_(2.0*M_PI), running_(false)
{}

DockingController::DockingController(float k_ro, float k_alpha, float k_beta) : k_ro_(k_ro), 
        k_alpha_(k_alpha), k_beta_(k_beta), robot_pose_(0.0, 0.0, 0.0), goal_(0.0, 0.0, 0.0), 
        lin_vel_sat_(2.0), ang_vel_sat_(2.0*M_PI), running_(false)
{}


DockingController::DockingController(float k_ro, float k_alpha, float k_beta, float lin_vel_sat, 
        float ang_vel_sat) : k_ro_(k_ro), k_alpha_(k_alpha), k_beta_(k_beta), robot_pose_(0.0, 0.0, 0.0), 
        goal_(0.0, 0.0, 0.0), lin_vel_sat_(lin_vel_sat), ang_vel_sat_(ang_vel_sat), running_(false)
{}


DockingController::~DockingController(){}




void DockingController::updateGains(float k_ro, float k_alpha, float k_beta)
{
    k_ro_ = k_ro;
    k_beta_ = k_beta;    
    k_alpha_ = k_alpha;
}


/*Pose controller described in Introduction of Autonomous Mobile Robots by Illah Reza
Noubakhsh and Roland Siegwart.*/

ControlSignal DockingController::computeControlSignal()
{

    if(!running_)
        return ControlSignal(0, 0);

    /*Both poses must be relative to the same frame (and cannot be null frame_id)*/
    if(robot_pose_.frame_id_ != goal_.frame_id_ || 
        robot_pose_.frame_id_.length() == 0 || goal_.frame_id_.length() == 0)
            throw ControlException("Robot pose and goal pose should be relative to the same non-null frame_id.");

    /*Convertion to the goal frame*/
    /*We have the transformation from goal to common frame [R | t], so its inverse is [R^T | - R^T*t] */
    /*This code is completely tested! Do NOT change!*/
    float theta = robot_pose_.theta_-goal_.theta_;
    float x_rp = cosf(goal_.theta_)*robot_pose_.x_+sinf(goal_.theta_)*robot_pose_.y_-cosf(goal_.theta_)*goal_.x_-sinf(goal_.theta_)*goal_.y_;
    float y_rp = -sinf(goal_.theta_)*robot_pose_.x_+cosf(goal_.theta_)*robot_pose_.y_+sinf(goal_.theta_)*goal_.x_-cosf(goal_.theta_)*goal_.y_;

    rho_ = sqrt(x_rp*x_rp+y_rp*y_rp);

    if(rho_ < 0.02)
        onDeadzone_ = true;
    else if(rho_ > 0.05)
	onDeadzone_ = false;

    /*rho is less than 2 cm*/
    if(onDeadzone_)
    {
        rho_ = 0.0;
	onDeadzone_ = true;
    }
        
    alpha_ = -theta+atan2f(-y_rp, -x_rp);

    /*Make sure that alpha is between in [-pi, pi]*/
    int n = (int) alpha_/(2.0*M_PI);
    if(n != 0)
        alpha_ = alpha_-n*2.0*M_PI;
    if(alpha_ > M_PI)
        alpha_ = alpha_-2.0*M_PI;
    else if(alpha_ < -M_PI)
        alpha_ = alpha_+2.0*M_PI;

    float v = k_ro_*rho_;


    /*Check if alpha belongs to I1 or I2 so that we can choose backward or forward parking*/
    if(!(alpha_ <= M_PI_2 && alpha_ > -M_PI_2))
    {
        /*If the robot is parking backward*/
        v = -v;
       
        if(alpha_ > M_PI_2)
        {
            alpha_ = alpha_ -M_PI;
        }else if(alpha_ <= -M_PI_2)
        {
            alpha_ = alpha_ + M_PI;
        }
        
    }

    beta_ = -theta-alpha_;
    /*Make sure that beta is between in [-pi, pi]*/
    n = (int) beta_/(2*M_PI);

    if(n != 0)
        beta_ =  beta_-n*2.0*M_PI;
    if(beta_ > M_PI)
        beta_ = beta_-2.0*M_PI;
    else if(beta_ < -M_PI)
        beta_ = beta_+2.0*M_PI;


    if(rho_ == 0)
    {
        w = k_alpha_*-theta;
    }else{
        w = k_alpha_*alpha_+k_beta_*beta_;
    }

    /*Noubakhsh and Siegwart do not consider actuation limits (saturating velocities).
    If one of the desired velocities is higher than what motor could deliver, we need to recompute
    the gains to ensure the strong stability condition (and get a predictable behavior)*/

    if(fabs(w) > ang_vel_sat_ || fabs(v) > lin_vel_sat_)
    {

        double w_ratio = fabs(ang_vel_sat_/w);
        double v_ratio = fabs(lin_vel_sat_/v);

        double scale = std::min(v_ratio, w_ratio);
        
        /*since the control signal results from a linear combination of the gains and they are all scaled
        in the same way, we just have to scale the control signals*/

        v = v*scale;
        w =  w*scale;
        
    }

    return ControlSignal(v, w);
}

Pose2D DockingController::getCurrentGoal()
{
    return goal_;
}

Pose2D DockingController::getCurrentPose()
{
    return robot_pose_;
}

void DockingController::updateRobotPose(float x, float y, float theta, std::string frame_id)
{
    robot_pose_.x_ = x;
    robot_pose_.y_ = y;
    robot_pose_.theta_ = theta;
    robot_pose_.frame_id_ = frame_id;
}

void DockingController::updateRobotPose(Pose2D newPose)
{
    robot_pose_ = newPose;
}


void DockingController::updateGoal(float x, float y, float theta, std::string frame_id)
{
    goal_.x_ = x;
    goal_.y_ = y;
    goal_.theta_ = theta;
    goal_.frame_id_ = frame_id;
}


void DockingController::updateGoal(Pose2D newGoal)
{
    goal_ = newGoal;
}

void DockingController::updateSaturations(float lin_vel_sat, float ang_vel_sat)
{
    lin_vel_sat_ = lin_vel_sat;
    ang_vel_sat_ = ang_vel_sat;
}


//END namespace docking_ctrl
}



