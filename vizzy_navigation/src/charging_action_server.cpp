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
					//onDeadzone = controller_.getDistanceError() < 0.3;
					ROS_INFO("Not on deadzone");
					controller_.updateGoal(goalPose);
					onDeadzone = true;
	      }else
				{
					ROS_INFO("On deadzone");
				}
				
	      
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
		ROS_INFO("At the docking move_base position");
		ros::Duration(10).sleep();

		estimator_.useFrontLaser();
		estimator_.enable();

		//Search for pattern with front laser
		feedback.state = feedback.SEARCHING_PATTERN;
		as_.publishFeedback(feedback);
		ROS_INFO("Searching for pattern with the fron laser and waiting for the median filter");
		ros::Duration(5).sleep();
		ROS_INFO("Pattern possibly found...");

		//Go to initial point using controller
		feedback.state = feedback.GOING_TO_INIT_POSE;
		as_.publishFeedback(feedback);

		ROS_INFO("Aligning robot with docking station using front laser detection");

		//We want to align the robot at the point that is 1.0m right in front of the docking station
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = -1.0;
	        pose.pose.position.y = 0.0;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 1.0; //The goal is rotated by 180deg from what the estimator is giving us
                pose.pose.orientation.w = 0.0;


		controlToGoalPose(pose, sampling_hz);
		estimator_.disable();
		ROS_INFO("Robot aligned");				

		// Start docking procedure

		estimator_.useBackLaser();
		
		ROS_INFO("Starting docking procedure - 1st intermediate point: 1.3m from docking station (should move forward)");
		feedback.state = feedback.DOCKING;
		as_.publishFeedback(feedback);
		estimator_.enable();
		ros::Duration(5).sleep();
		estimator_.disable();

		pose.pose.position.x = -1.3;
		pose.pose.orientation.z = 1.0; //The goal is rotated by 180deg from what the estimator is giving us

		estimator_.enable();
		controlToGoalPose(pose, sampling_hz);
		ros::Duration(5).sleep();
		estimator_.disable();
		
		
		ROS_INFO("Finishing docking procedure - final point: 0.5m from docking station (going backwards)");

		pose.pose.position.x = -0.5;
		estimator_.enable();
		controlToGoalPose(pose, sampling_hz);
		estimator_.disable();

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


		//Undocking going 1m forward
		ROS_INFO("Undocking...");

		geometry_msgs::PoseStamped goalOutside;
		goalOutside.header.frame_id = "base_footprint";
		goalOutside.pose.position.x = -1.0;
		goalOutside.pose.position.y = 0.0;
		goalOutside.pose.position.z = 0.0;

		goalOutside.pose.orientation.x = 0.0;
		goalOutside.pose.orientation.y = 0.0;
		goalOutside.pose.orientation.z = 0.0;
		goalOutside.pose.orientation.w = 1.0;

		controller_.updateGoal(goalOutside);

		bool onPoint = false;

		controller_.enableControl();
		while(!onPoint)
		{
			if(as_.isPreemptRequested())
			{
				result.result = result.STOPPED_FAILED;
				as_.setAborted(result);
				controller_.disableControl();
				ROS_ERROR("Undocking preempted. Stopping robot!");
				return;
			}else
			{
				controller_.run();

				if(controller_.getDistanceError() < 0.05 && controller_.getOrientationError() < 0.05)
				{
					controller_.disableControl();
					onPoint = true;
				}
			}

			sampling_hz.sleep();
			
		}

		if(true)
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

