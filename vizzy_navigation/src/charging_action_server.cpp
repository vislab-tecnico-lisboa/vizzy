#include <charging_action_server.hpp>
#include <vizzy_msgs/ChargeFeedback.h>
#include <vizzy_msgs/BatteryChargingState.h>
#include <vizzy_msgs/MotorsShutdown.h>
#include <vizzy_msgs/ArmDown.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void ChargingActionServer::controlToGoalPose(geometry_msgs::PoseStamped & pose, ros::Rate & sampling_hz,  bool onDeadzone)
{
	sampling_hz.reset();
	bool onPoint = false;
	vizzy_msgs::ChargeResult result;
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
				//	ROS_INFO("On deadzone");
				}


	      controller_.enableControl();
	      controller_.run();

	      //If we are close enough to the goal, we are done (1cm error and )
	      ROS_ERROR_STREAM("dist error: " << controller_.getDistanceError() << ", ang error: " << controller_.getOrientationError());
	      if(controller_.getDistanceError() < 0.04 && fabs(controller_.getOrientationError()) < 0.02)
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
	vizzy_msgs::ChargeFeedback feedback;
	vizzy_msgs::ChargeResult result;

  
	//Charge goal
	if(goal->goal == vizzy_msgs::ChargeGoal::CHARGE)
	{
		ROS_INFO("Received new goal: Charge!");

		estimator_.useFrontLaser();
		estimator_.enable();

		//Search for pattern with front laser
		feedback.state = feedback.SEARCHING_PATTERN;
		as_.publishFeedback(feedback);
		ROS_INFO("Searching for pattern with the front laser and waiting for the median filter");
		ROS_INFO("Pattern possibly found...");

		ros::Duration(10).sleep();
		estimator_.disable();
		//Go to initial point using controller
		feedback.state = feedback.GOING_TO_INIT_POSE;
		as_.publishFeedback(feedback);

		ROS_INFO("Aligning robot with docking station using front laser detection");

		//We want to align the robot at the point that is 1.0m right in front of the docking station
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = -1.0;
	    pose.pose.position.y = -0.04;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 1.0; //The goal is rotated by 180deg from what the estimator is giving us
		pose.pose.orientation.w = 0.0;


		controlToGoalPose(pose, sampling_hz);
		ROS_INFO("Robot aligned");				

		// Start docking procedure

		estimator_.useBackLaser();
		
		ROS_INFO("Starting docking procedure - 1st intermediate point: 1.3m from docking station (should move forward)");
		feedback.state = feedback.DOCKING;
		as_.publishFeedback(feedback);
		estimator_.enable();
		ros::Duration(10).sleep();
		estimator_.disable();

		pose.pose.position.x = -1.3;
        pose.pose.position.y = -0.04;
		pose.pose.orientation.z = 1.0; //The goal is rotated by 180deg from what the estimator is giving us

		controlToGoalPose(pose, sampling_hz);
		
		
		ROS_INFO("Finishing docking procedure - final point: 0.3m from docking station (going backwards) to the robot base center");

		estimator_.enable();
		ros::Duration(10).sleep();
		estimator_.disable();
		pose.pose.position.x = -0.7;
		
		controlToGoalPose(pose, sampling_hz);


		ROS_INFO("Open loop backward mega speed!");
		

		ros::Rate hz(0.5);
		int loops = 0;
		geometry_msgs::Twist cmd_vel;
		
		while(loops < 6)
		{
                  cmd_vel.linear.x = -0.355;
                  cmd_vel.angular.z = 0;
                  cmd_pub_.publish(cmd_vel);
		  hz.sleep();
		  loops++;
		}

		cmd_vel.linear.x = 0.0;
		cmd_pub_.publish(cmd_vel);

		//Done - Charging
		feedback.state = feedback.CHARGING;
		as_.publishFeedback(feedback);
		vizzy_msgs::BatteryChargingState my_msg;
		if (charging_state_client_.call(my_msg)){
			if (my_msg.response.battery_charging_state){
				// Call arms idle service to charge faster
				result.result = result.CHARGE_SUCCESS;
				vizzy_msgs::MotorsShutdown arms_idle_msg;
				arms_idle_msg.request.shutdown_request = arms_idle_msg.request.SHUTDOWN;
				if (arms_idle_client.call(arms_idle_msg)){
					if (arms_idle_msg.response.shutdown_reply == arms_idle_msg.response.SUCCESS){
						ROS_INFO("Done - charging and arm motors are idle");
					}
					else{
						ROS_ERROR("Done - charging but the arm motors are not idle");		
					}
				}
				else{
					ROS_ERROR("Done - charging but the arm motors service is not running");	
				}
			}
			else{
				result.result = result.CHARGE_FAILED;
				ROS_ERROR("Failed to check the charging state. Unknown charging state!");
			}
		}
		else{
			ROS_ERROR("Failed to check the charging state. Stopping robot!");
			result.result = result.CHARGE_FAILED;
		}
		as_.setSucceeded(result);
		return;

	} //Undock goal
	else if(goal->goal == vizzy_msgs::ChargeGoal::STOP_CHARGE)
	{
		ROS_INFO("Received new goal: Stop Charge!");


		//Undocking going 1m forward
		ROS_INFO("Going a bit forward in open loop...");

		ros::Rate hz(0.5);
		int loops = 0;
		geometry_msgs::Twist cmd_vel;

		while(loops < 2)
		{
                  cmd_vel.linear.x = 0.355;
                  cmd_vel.angular.z = 0;
                  cmd_pub_.publish(cmd_vel);
		  hz.sleep();
		  loops++;
		}

		ROS_INFO("Navigating 0.5m forward with planner...");
		geometry_msgs::PoseStamped goalOutside;
		goalOutside.header.frame_id = "base_footprint";
		goalOutside.pose.position.x = 0.3;
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
		vizzy_msgs::BatteryChargingState my_msg;
		if (onPoint){
			if (charging_state_client_.call(my_msg)){
				if (!my_msg.response.battery_charging_state){
					result.result = result.STOPPED;
					vizzy_msgs::MotorsShutdown arms_idle_msg;
					arms_idle_msg.request.shutdown_request = arms_idle_msg.request.TURNON;
					if (arms_idle_client.call(arms_idle_msg)){
						if (arms_idle_msg.response.shutdown_reply == arms_idle_msg.response.SUCCESS){
							vizzy_msgs::ArmDown arm_down_msg;
							arm_down_msg.request.robot_type=arm_down_msg.request.REAL;
							arm_down_msg.request.arm_down_request=arm_down_msg.request.LEFT;
							if (arms_home_client.call(arm_down_msg)){
								if (arm_down_msg.response.arm_down_reply==arm_down_msg.response.SUCCESS){
									ROS_INFO("Left arm home position done");		
								}
								else{
									ROS_ERROR("Not able to reach left arm home position");
								}
							}
							arm_down_msg.request.arm_down_request=arm_down_msg.request.RIGHT;
							if (arms_home_client.call(arm_down_msg)){
								if (arm_down_msg.response.arm_down_reply==arm_down_msg.response.SUCCESS){
									ROS_INFO("Right arm home position done");		
								}
								else{
									ROS_ERROR("Not able to reach right arm home position");
								}
							}
							ROS_INFO("Done - undocked successfully and arm motors are on");
						}
						else{
							ROS_ERROR("Done - undocked successfully but the arm motors are not on");		
						}
					}
					else{
						ROS_ERROR("Done - undocked successfully but the arm motors service is not running");	
					}
				}
				else{
					result.result = result.STOPPED_FAILED;
					ROS_ERROR("Failed to check the charging state. Unknown charging state!");
				}
			}
			else{
				ROS_ERROR("Failed to check the charging state. Unknown charging state!");
				result.result = result.STOPPED_FAILED;
			}
			as_.setSucceeded(result);
			return;
		/*if(true)
		{
		  ROS_INFO("Undocked successfully!");
		  result.result = result.STOPPED;
		}*/
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
    estimator_(nh), tfBuffer_(), tfListener_(tfBuffer_), as_(nh_, name, false)
{
    as_.registerGoalCallback(boost::bind(&ChargingActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&ChargingActionServer::preemptCB, this));
   
    cmd_pub_ =  nh_.advertise<geometry_msgs::Twist>("/vizzy/cmd_vel", 1 );
    charging_state_client_ = nh_.serviceClient<vizzy_msgs::BatteryChargingState>("battery_charging_state");
	arms_idle_client = nh_.serviceClient<vizzy_msgs::MotorsShutdown>("armMotorsIdle");
	arms_home_client = nh_.serviceClient<vizzy_msgs::ArmDown>("armDown");

    as_.start();
}

ChargingActionServer::~ChargingActionServer()
{}

