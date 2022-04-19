/*Copyright 2021, Joao Avelino, All rights reserved.*/

#include <mapless_nav/mapless_navigator.hpp>

using namespace std;


MaplessNavigator::MaplessNavigator(ros::NodeHandle &nh) : nh_(nh), nPriv_("~"), tfListener_(tfBuffer_),
                    tf2Filter_(poseSub_, tfBuffer_, "/base_footprint", 10, 0), 
					as_(nh_, "mapless_action", false),
					controller_(0.5, 1.333, -0.25),
					obs_avoider_(nh, nPriv_){

	poseSub_.subscribe(nh_, "/mapless_goal", 1);
	tf2Filter_.registerCallback( boost::bind(&MaplessNavigator::goalCallback, this, _1) );
	cmdPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 

	action_goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/mapless_action/goal", 1);

	f_ = boost::bind(&MaplessNavigator::dynamic_rec_callback, this, _1, _2);
	server_.setCallback(f_);

	as_.registerGoalCallback(boost::bind(&MaplessNavigator::actionGoalCB, this));
	as_.registerPreemptCallback(boost::bind(&MaplessNavigator::actionPreemptCB, this));

	as_.start();
	
	//advertise a service for clearing the costmaps (as in the move_base package)
	clear_costmaps_srv_ = nPriv_.advertiseService("clear_costmaps", &MaplessNavigator::clearCostmapsService, this);


}

bool MaplessNavigator::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(obs_avoider_.mLocalMap->getCostmap()->getMutex()));
    obs_avoider_.mLocalMap->resetLayers();

    return true;
}

void MaplessNavigator::dynamic_rec_callback(vizzy_navigation::MaplessConfig &config, uint32_t level)
{

	ROS_INFO("Reconfigure Request: k_ro: %f,  k_alpha: %f, k_beta: %f, frame_id: %s, lin_sat: %f (m/s), ang_sat: %f (rad/s),\
		linvel_min: %f (m/s), angvel_min: %f (rad/s)", 
	    config.k_ro, config.k_alpha, config.k_beta, config.common_frame.c_str(), config.lin_vel_sat, config.ang_vel_sat,
		config.linvel_min, config.angvel_min);

	ROS_INFO("Reconfigure Request: max_free_space: %f,  safety_decay: %f, safety_weight: %d, \
		conformance_weight: %d, continue_weight: %d, escape_weight: %d",
	    config.max_free_space, config.safety_decay, config.safety_weight, config.conformance_weight,
		config.continue_weight, config.escape_weight);

	common_frame_ = config.common_frame;
	controller_.updateGains(config.k_ro, config.k_alpha, config.k_beta);
	controller_.updateSaturations(config.lin_vel_sat, config.ang_vel_sat);
	linvel_min_ = config.linvel_min;
	angvel_min_ = config.angvel_min;

	tf2Filter_.setTargetFrame(common_frame_);

	obs_avoider_.mMaxFreeSpace = config.max_free_space;
	obs_avoider_.mSafetyDecay = config.safety_decay;
	obs_avoider_.mSafetyWeight = config.safety_weight;
	obs_avoider_.mConformanceWeight = config.conformance_weight;
	obs_avoider_.mContinueWeight = config.continue_weight;
	obs_avoider_.mEscapeWeight = config.escape_weight;
	obs_avoider_.mMaxVelocity = config.lin_vel_sat;
	obs_avoider_.mDriveMode = config.mode;
	obs_avoider_.mMaxLinearVelocity = config.lin_vel_sat;
	obs_avoider_.mMaxAngularVelocity = config.ang_vel_sat;

}

MaplessNavigator::~MaplessNavigator(){


    std::cout << "Mapless navigator being destroyed. Sending zero velocity!" << std::endl;

    disableControl();

    std::cout << "Zero velocity sent to: " << cmdPub_.getTopic() << std::endl;

}

//Updaters the current goal
void MaplessNavigator::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *msg;

    action_goal_pub_.publish(action_goal);

}


void MaplessNavigator::doControlBase()
{

	if(!controller_.running_)
		return;

	if(!ros::ok())
	{
		ROS_INFO("Preempting mapless navigation for shutdown!");
		as_.setPreempted();
		disableControl();
		return;
	}

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
		
		if(as_.isActive())
			as_.setSucceeded();
		return;
	}

	//If the obstacle avoidance module signals that the robot is stuck, stop control.
	//and say it failed

	if(obs_avoider_.rStuck)
	{
		disableControl();
		if(as_.isActive())
			as_.setAborted();
		return;
	}

	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = cont_signal.linear_vel_;
	cmd_vel.angular.z = cont_signal.angular_vel_;

	obs_avoider_.receiveCommand(cmd_vel);
	cmd_vel = obs_avoider_.computeVelocity();

	cmdPub_.publish(cmd_vel);

	if(as_.isActive())
	{
		feedback_.base_position = robot_pose_;
		as_.publishFeedback(feedback_);
	}

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
	obs_avoider_.rStuck = false;
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

void MaplessNavigator::actionGoalCB()
{

	auto goal = as_.acceptNewGoal();

    try 
    {
      tfBuffer_.transform(goal->target_pose, current_goal_, common_frame_);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
	  as_.setAborted();
    }
    
    mapless_controller::Pose2D goalPose = makeTransform(goal->target_pose, common_frame_);
    controller_.updateGoal(goalPose);
    last_update_ = goal->target_pose.header.stamp;

	enableControl();

}

void MaplessNavigator::actionPreemptCB()
{
	if(as_.isActive())
		as_.setPreempted();

	disableControl();
	return;

}