/*Copyright 2019, Joao Avelino, All rights reserved.*/

#ifndef DOCKING_CONTROLLER_HPP_
#define DOCKING_CONTROLLER_HPP_
#include <string>


namespace docking_ctrl{

class ControlException : public std::exception
{   

public:

    ControlException(const std::string& msg) : m_msg(msg) {}

    virtual const char* what() const throw()
    {
        return m_msg.c_str();
    }

    const std::string m_msg;
};

class Pose2D
{
public:
    Pose2D(){}
    Pose2D(float x, float y, float theta) : 
        x_(x), y_(y), theta_(theta){}
    
    Pose2D(float x, float y, float theta, std::string frame_id) : 
        x_(x), y_(y), theta_(theta), frame_id_(frame_id){}
    float x_;
    float y_;
    float theta_;
    std::string frame_id_;
};

class ControlSignal
{
public:
    ControlSignal() : linear_vel_(0), angular_vel_(0){};
    ControlSignal(float v, float w) : linear_vel_(v), angular_vel_(w){};
    float linear_vel_;
    float angular_vel_;
};


class DockingController
{
private:

    float k_ro_;
    float k_alpha_;
    float k_beta_;
    Pose2D robot_pose_;
    Pose2D goal_;
    float lin_vel_sat_;
    float ang_vel_sat_;
    



public:
    DockingController(float k_ro, float k_alpha, float k_beta, Pose2D goal, Pose2D robot_pose);
    DockingController(float k_ro, float k_alpha, float k_beta, float lin_vel_sat, float ang_vel_sat);
    DockingController(float k_ro, float k_alpha, float k_beta);
    ~DockingController();
    void updateGains(float k_ro, float k_alpha, float k_beta);
    void updateSaturations(float lin_vel_sat_, float ang_vel_sat_);
    ControlSignal computeControlSignal();
    Pose2D getCurrentPose();
    Pose2D getCurrentGoal();

    
    void updateRobotPose(float x, float y, float theta, std::string frame_id);
    void updateRobotPose(Pose2D newPose);
    void updateGoal(float x, float y, float theta, std::string frame_id);
    void updateGoal(Pose2D newGoal);

    bool running_;

};

//END namespace docking_ctrl
}

#endif