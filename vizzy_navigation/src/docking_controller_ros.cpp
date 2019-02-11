/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <docking_controller_ros.hpp>

docking_ctrl::Pose2D DockingControllerROS::makeTransform(geometry_msgs::PoseStamped pose, std::string target_frame, ros::Time fromTime)
{
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped onCommon;

    try{
    transformStamped = tfBuffer_.lookupTransform(target_frame, ros::Time(0), 
                                pose.header.frame_id, fromTime, target_frame);
    tf2::doTransform(pose, onCommon, transformStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return docking_ctrl::Pose2D();
    }
    
    tf2::Transform goalPoseTF;
    tf2::fromMsg(onCommon.pose, goalPoseTF);
    tf2::Matrix3x3 rotMat = goalPoseTF.getBasis();
    double yaw, pitch, roll;
    rotMat.getEulerYPR(yaw, pitch, roll);
    
    return docking_ctrl::Pose2D(onCommon.pose.position.x, onCommon.pose.position.y, yaw, common_frame_);

}

/*Controller initialized with default parameters*/
DockingControllerROS::DockingControllerROS(ros::NodeHandle nh) : 
    nh_(nh), controller_(0.5, 1.333, -0.25), tfBuffer_(), tfListener_(tfBuffer_)
{
  
  f_ = boost::bind(&DockingControllerROS::dynamic_rec_callback, this, _1, _2);
  server_.setCallback(f_);

  //Command topic
  pub_ =  nh_.advertise<geometry_msgs::Twist>("/vizzy/cmd_vel", 1 );
  
  /*DEBUG - Goal topic*/
  sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("goal_topic", 1, &DockingControllerROS::goalCallback, this);
  
}

void DockingControllerROS::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    docking_ctrl::Pose2D goalPose = makeTransform(*msg, common_frame_);
    controller_.updateGoal(goalPose);
    last_update_ = msg->header.stamp;
    controller_.running_ = true;

}

void DockingControllerROS::updateGoal(geometry_msgs::PoseStamped& goal)
{

    docking_ctrl::Pose2D goalPose = makeTransform(goal, common_frame_);
    controller_.updateGoal(goalPose);
    last_update_ = goal.header.stamp;
    controller_.running_ = true;

}

double DockingControllerROS::getDistanceError()
{
  return controller_.rho_;
}


double DockingControllerROS::getOrientationError()
{
  return controller_.beta_;
}



DockingControllerROS::~DockingControllerROS(){}

void DockingControllerROS::dynamic_rec_callback(vizzy_navigation::DockingConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: k_ro: %f,  k_alpha: %f, k_beta: %f, frame_id: %s, lin_sat: %f (m/s), ang_sat: %f (rad/s)", 
            config.k_ro, config.k_alpha, config.k_beta, config.common_frame.c_str(), config.lin_vel_sat, config.ang_vel_sat);
  
  common_frame_ = config.common_frame;
  controller_.updateGains(config.k_ro, config.k_alpha, config.k_beta);
  controller_.updateSaturations(config.lin_vel_sat, config.ang_vel_sat);
}

void DockingControllerROS::disableControl()
{
  controller_.running_ = false;
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub_.publish(cmd_vel);
}

void DockingControllerROS::enableControl()
{
  controller_.running_ = true;
}


void DockingControllerROS::run()
{

  if(!controller_.running_)
    return;

  /*Update current robot pose on common frame*/

    geometry_msgs::PoseStamped robotPose;
    robotPose.header.frame_id = "base_footprint";
    robotPose.pose.orientation.w = 1.0;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0;
    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;

    docking_ctrl::Pose2D robotPose2D = makeTransform(robotPose, common_frame_);
    controller_.updateRobotPose(robotPose2D);

    /*Update current goal on common frame*/
    geometry_msgs::PoseStamped goalPose;
    docking_ctrl::Pose2D goalPose2D = controller_.getCurrentGoal();
    goalPose.header.frame_id = goalPose2D.frame_id_;
    goalPose.pose.position.x = goalPose2D.x_;
    goalPose.pose.position.y = goalPose2D.y_;
    goalPose.pose.position.z = 0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, goalPose2D.theta_);
    geometry_msgs::Quaternion quatmsg;
    goalPose.pose.orientation = tf2::toMsg(quat);
    goalPose2D = makeTransform(goalPose, common_frame_, last_update_);
    controller_.updateGoal(goalPose2D);


  /*Compute the control signal*/

  docking_ctrl::ControlSignal cont_signal;
  
  try{
    cont_signal = controller_.computeControlSignal();
  }catch(docking_ctrl::ControlException& e){
    ROS_ERROR("Control Exception: %s", e.what());
    cont_signal.linear_vel_ = 0.0;
    cont_signal.angular_vel_ = 0.0;
  }

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = cont_signal.linear_vel_;
  cmd_vel.angular.z = cont_signal.angular_vel_;

  pub_.publish(cmd_vel);

}