/*Copyright 2021, Joao Avelino, All rights reserved.*/

#include <mapless_nav/mapless_navigator.hpp>

using namespace std;


MaplessNavigator::MaplessNavigator(ros::NodeHandle &nh) : nh_(nh), nPriv_("~"), tfListener_(tfBuffer_),
                    tf2Filter_(poseSub_, tfBuffer_, "/base_footprint", 10, 0), controller_(0.5, 1.333, -0.25),
					obs_avoider_(nh, nPriv_){

  poseSub_.subscribe(nh_, "/mapless_goal", 1);
  tf2Filter_.registerCallback( boost::bind(&MaplessNavigator::goalCallback, this, _1) );
  cmdPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 

	f_ = boost::bind(&MaplessNavigator::dynamic_rec_callback, this, _1, _2);
	server_.setCallback(f_);

}

void MaplessNavigator::dynamic_rec_callback(vizzy_navigation::MaplessConfig &config, uint32_t level)
{

	ROS_INFO("Reconfigure Request: k_ro: %f,  k_alpha: %f, k_beta: %f, frame_id: %s, lin_sat: %f (m/s), ang_sat: %f (rad/s),\
		linvel_min: %f (m/s), angvel_min: %f (rad/s)", 
	    config.k_ro, config.k_alpha, config.k_beta, config.common_frame.c_str(), config.lin_vel_sat, config.ang_vel_sat,
		config.linvel_min, config.angvel_min);

	common_frame_ = config.common_frame;
	controller_.updateGains(config.k_ro, config.k_alpha, config.k_beta);
	controller_.updateSaturations(config.lin_vel_sat, config.ang_vel_sat);
	linvel_min_ = config.linvel_min;
	angvel_min_ = config.angvel_min;

	tf2Filter_.setTargetFrame(common_frame_);

}

MaplessNavigator::~MaplessNavigator(){


    std::cout << "Mapless navigator being destroyed. Sending zero velocity!" << std::endl;

    disableControl();

    std::cout << "Zero velocity sent to: " << cmdPub_.getTopic() << std::endl;

}

//Updaters the current goal
void MaplessNavigator::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    try 
    {
      tfBuffer_.transform(*msg, current_goal_, common_frame_);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
    
    mapless_controller::Pose2D goalPose = makeTransform(*msg, common_frame_);
    controller_.updateGoal(goalPose);
    last_update_ = msg->header.stamp;

	enableControl();

}


void MaplessNavigator::doControlBase()
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

	mapless_controller::Pose2D robotPose2D = makeTransform(robotPose, common_frame_);
	controller_.updateRobotPose(robotPose2D);

	/*Update current goal on common frame*/
	geometry_msgs::PoseStamped goalPose;
	mapless_controller::Pose2D goalPose2D = controller_.getCurrentGoal();
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
	mapless_controller::ControlSignal cont_signal;

	try{
	cont_signal = controller_.computeControlSignal();
	}catch(mapless_controller::ControlException& e){
	ROS_ERROR("Control Exception: %s", e.what());
	cont_signal.linear_vel_ = 0.0;
	cont_signal.angular_vel_ = 0.0;
	}

	//If both linear and angular velocities are too low for the segway to move, then stop
	//the controller and assume that we achieved the goal

	if(fabs(cont_signal.linear_vel_) < linvel_min_ && fabs(cont_signal.angular_vel_) < angvel_min_)
	{
		disableControl();
		return;
	}

	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = cont_signal.linear_vel_;
	cmd_vel.angular.z = cont_signal.angular_vel_;

	obs_avoider_.receiveCommand(cmd_vel);
	cmd_vel = obs_avoider_.computeVelocity();

	cmdPub_.publish(cmd_vel);

}

mapless_controller::Pose2D MaplessNavigator::makeTransform(geometry_msgs::PoseStamped pose, std::string target_frame, ros::Time fromTime)
{
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped onCommon;

    try{
    transformStamped = tfBuffer_.lookupTransform(target_frame, ros::Time(0), 
                                pose.header.frame_id, fromTime, target_frame);
    tf2::doTransform(pose, onCommon, transformStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Make transform: %s",ex.what());
      ros::Duration(1.0).sleep();
      return mapless_controller::Pose2D();
    }
    
    tf2::Transform goalPoseTF;
    tf2::fromMsg(onCommon.pose, goalPoseTF);
    tf2::Matrix3x3 rotMat = goalPoseTF.getBasis();
    double yaw, pitch, roll;
    rotMat.getEulerYPR(yaw, pitch, roll);
    
    return mapless_controller::Pose2D(onCommon.pose.position.x, onCommon.pose.position.y, yaw, common_frame_);

}


void MaplessNavigator::disableControl()
{
	controller_.running_ = false;
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	cmdPub_.publish(cmd_vel);
}

void MaplessNavigator::enableControl()
{
	controller_.running_ = true;
}

void MaplessNavigator::updateGoal(geometry_msgs::PoseStamped& goal)
{
	mapless_controller::Pose2D goalPose = makeTransform(goal, common_frame_);
	controller_.updateGoal(goalPose);
	last_update_ = goal.header.stamp;
	controller_.running_ = true;
}

double MaplessNavigator::getDistanceError()
{
	return controller_.rho_;
}


double MaplessNavigator::getOrientationError()
{
	return controller_.w;
}