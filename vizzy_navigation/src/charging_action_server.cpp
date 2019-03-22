#include <charging_action_server.hpp>
#include <vizzy_navigation/ChargeFeedback.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void ChargingActionServer::controlToGoalPose(geometry_msgs::PoseStamped & pose, ros::Rate & sampling_hz,  bool onDeadzone)
{
	sampling_hz.reset();

	bool onPoint = false;
	vizzy_navigation::ChargeResult result;
	// control
	while(!onPoint)
	{ 
	  //Cancel everything. Stop the robot
	  if(as_.isPreemptRequested())
	  {
	      result.result = result.CHARGE_FAILED;
	      as_.setAborted(result);
	      controller_.disableControl();
	      estimator_.disable();
	      ROS_ERROR("Docking preempted. Stopping robot!");
	      return;
	  }else{
	      //********** Update the goal and control the robot **************

	      //Create the transform from laser to docking station
	      geometry_msgs::PoseStamped station = estimator_.getPatternPose();

	      tf2::Stamped<tf2::Transform> transf;
	      tf2::fromMsg(station, transf);

	      geometry_msgs::TransformStamped dpTFStamped;
	      dpTFStamped = tf2::toMsg(transf);
	      
	      geometry_msgs::PoseStamped goalPose;
	      pose.header = station.header;

	      
	      tf2::doTransform(pose, goalPose, dpTFStamped);
	      goalPose.header = station.header;

	      //Do a control step
	      if(!onDeadzone)
	      {
		onDeadzone = controller_.getDistanceError() < 0.3;
		controller_.updateGoal(goalPose);
	      }
              /*else
              {
		ROS_INFO("On first deadzone");
	      }*/
	      
	      controller_.enableControl();
	      controller_.run();

	      //If we are close enough to the goal, we are done (1cm error and )
	      if(controller_.getDistanceError() < 0.05 && controller_.getOrientationError() < 0.05)
	      {
		  controller_.disableControl();
		  estimator_.disable();
		  onPoint = true;
	      }
	  }
	  sampling_hz.sleep();
	}	
}

void ChargingActionServer::goalCallback()
{
	ros::Rate sampling_hz(10);
	//Get the goal
	auto goal = as_.acceptNewGoal();
	vizzy_navigation::ChargeFeedback feedback;
	vizzy_navigation::ChargeResult result;

	//Cancel all move base goals. It's time to charge
	move_base_client_.cancelAllGoals();
  
	//Charge goal
	if(goal->goal == vizzy_navigation::ChargeGoal::CHARGE)
	{
		ROS_INFO("Received new goal: Charge!");

		//Navigate to docking station
		feedback.state = feedback.NAVIGATING_TO_STATION;
		as_.publishFeedback(feedback);

		ROS_INFO("Navigating to docking position");
		move_base_client_.sendGoal(goal_msg);

		move_base_client_.waitForResult();

		if(move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
		    ROS_INFO("Successfully navigated to docking station");
		}
		else
		{
		    ROS_INFO("I can not navigate to the docking station for some reason!");
		    result.result = result.CHARGE_FAILED;
		    as_.setAborted(result);
		    return;
		}
		ROS_INFO("At the docking position");
		ros::Duration(10).sleep();


		//Search for pattern
		feedback.state = feedback.SEARCHING_PATTERN;
		as_.publishFeedback(feedback);
		ROS_INFO("Searching for pattern (not doing anything now)...");
		ROS_INFO("Pattern found (not really. this is a fake msg for now)...");

		//Go to initial point using controller
		feedback.state = feedback.GOING_TO_INIT_POSE;
		as_.publishFeedback(feedback);

		ros::Duration(5).sleep();

		ROS_INFO_STREAM("DUMMY: estimate position of the front laser");
		estimator_.enable(); // HERE SHOULD estimate the position of the pattern using the front laser
		estimator_.disable();
		ROS_INFO_STREAM("DUMMY: done estimation position of the front laser");

		/* start control point routine */
		ROS_INFO("Rotating...");
		//We want to align the robot at the point that is 1.0m right in front of the docking station
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id="base_footprint";
		//pose.pose.position = goal_msg.target_pose.pose.position;
		//Eigen::Affine3d initial_pose;
		//tf::poseMsgToEigen (goal_msg.target_pose.pose,initial_pose);

		//The goal is translated and rotated by....
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 1.0; 
		pose.pose.orientation.w = 0.0;
		/*Eigen::Affine3d rotation_pose;
		tf::poseMsgToEigen (goal_msg.target_pose.pose,rotation_pose);
		Eigen::Affine3d final_pose;
		final_pose=rotation_pose*initial_pose;
		tf::poseEigenToMsg (final_pose, pose.pose);*/
		//pose.pose.orientation.x = 0.0;
		//pose.pose.orientation.y = 0.0;
		//pose.pose.orientation.z = 0.7071068; //The goal is rotated by 180deg from what the estimator is giving us
		//pose.pose.orientation.w = 0.7071068;
		controller_.updateGoal(pose);

		bool onPoint=false;
		while(!onPoint)
		{ 
			controller_.run();
			controller_.enableControl();
			if(controller_.getDistanceError() < 0.05 && fabs(controller_.getOrientationError()) < 0.05)
			{
				controller_.disableControl();
				estimator_.disable();
				onPoint = true;
				std::cout << "entrei" << std::endl;
			}
			std::cout << controller_.getOrientationError() << std::endl;
		}

		controller_.run();
		ros::Duration(5).sleep();
		ROS_INFO("Finished rotating");
		/* end control point routine */

		// Start docking procedure
		feedback.state = feedback.DOCKING;
		as_.publishFeedback(feedback);

		bool docked = false;

		/* start docking routine */
		ROS_INFO("Started docking routine");
		pose.pose.position = goal_msg.target_pose.pose.position;
		pose.pose.position.x = goal_msg.target_pose.pose.position.x-3.0;
		estimator_.enable();
		controlToGoalPose(pose, sampling_hz);
		estimator_.disable();
		ros::Duration(5).sleep();
		ROS_INFO("Finished docking routine");
		/* end docking routine */

		//Done - Charging
		feedback.state = feedback.CHARGING;
		as_.publishFeedback(feedback);
		result.result = result.CHARGE_SUCCESS;
		as_.setSucceeded(result);
		ROS_INFO("Done - charging");
		return;

	} //Undock goal
	else if(goal->goal == vizzy_navigation::ChargeGoal::STOP_CHARGE)
	{
		ROS_INFO("Received new goal: Stop Charge!");

		//For now just go 1m forward... Use move_base because its easy... ISTO NÃO PODE FICAR!
		//OS RECOVERY BEHAVIORS VAO FAZER COM QUE ELE RODE SOBRE SI PROPRIO DENTRO DA DOCKING STATION!!!

		move_base_msgs::MoveBaseGoal goalOutside;
		goalOutside.target_pose.header.frame_id = "base_footprint";
		goalOutside.target_pose.pose.position.x = -1.0;
		goalOutside.target_pose.pose.position.y = 0.0;
		goalOutside.target_pose.pose.position.z = 0.0;

		goalOutside.target_pose.pose.orientation.x = 0.0;
		goalOutside.target_pose.pose.orientation.y = 0.0;
		goalOutside.target_pose.pose.orientation.z = 0.0;
		goalOutside.target_pose.pose.orientation.w = 1.0;

		move_base_client_.sendGoal(goalOutside);
		move_base_client_.waitForResult();

		if(move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
		  ROS_INFO("Undocked successfully!");
		  result.result = result.STOPPED;
		}
		else
		{
		  ROS_INFO("I can not navigate ouside of the docking station for some reason!");
		  result.result = result.STOPPED_FAILED;
		  as_.setAborted(result);
		  return;
		}
	} //Invalid goal
	else
	{
		ROS_ERROR("Charging action server: Invalid goal!");
	}
}

void ChargingActionServer::preemptCB()
{
	ROS_INFO("Docking: Preempted");
	as_.setPreempted();
}

void ChargingActionServer::run()
{}


ChargingActionServer::ChargingActionServer(ros::NodeHandle nh, std::string name) : controller_(nh), n_priv("~"),
    estimator_(nh), tfBuffer_(), tfListener_(tfBuffer_), as_(nh_, name, false), move_base_client_("move_base", true)
{
    as_.registerGoalCallback(boost::bind(&ChargingActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&ChargingActionServer::preemptCB, this));

    ROS_INFO("Waiting for move base action server");
    move_base_client_.waitForServer();
    ROS_INFO("Found move_base action server");
    
    goal_msg.target_pose.header.stamp = ros::Time::now();

    n_priv.param<std::string>("docking_frame_id", goal_msg.target_pose.header.frame_id, "map");
    n_priv.param("docking_pos_x", goal_msg.target_pose.pose.position.x, 0.0);
    n_priv.param("docking_pos_y", goal_msg.target_pose.pose.position.y, 0.0);
    n_priv.param("docking_pos_z", goal_msg.target_pose.pose.position.z, 0.0);

    n_priv.param("docking_orient_x", goal_msg.target_pose.pose.orientation.x, 0.0);
    n_priv.param("docking_orient_y", goal_msg.target_pose.pose.orientation.y, 0.0);
    n_priv.param("docking_orient_z", goal_msg.target_pose.pose.orientation.z, 0.0);
    n_priv.param("docking_orient_w", goal_msg.target_pose.pose.orientation.w, 1.0);

    as_.start();
}

ChargingActionServer::~ChargingActionServer()
{}

