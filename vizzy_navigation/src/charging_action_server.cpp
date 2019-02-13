#include <charging_action_server.hpp>
#include <vizzy_navigation/ChargeFeedback.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void ChargingActionServer::goalCallback()
{
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

        ROS_INFO("Navigating to docking station");
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

      //Search for pattern
      feedback.state = feedback.SEARCHING_PATTERN;
      as_.publishFeedback(feedback);
      ROS_INFO("Searching for pattern (not doing anything now)...");
      ROS_INFO("Pattern found (not really. this is a fake msg for now)...");

      //Go to initial point using controller
      feedback.state = feedback.GOING_TO_INIT_POSE;
      as_.publishFeedback(feedback);
      ROS_INFO("Aligning robot with docking station");

      //We want to align the robot at the point that is 1.0m right in front of the docking station
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = -1.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 1.0; //The goal is rotated by 180deg from what the estimator is giving us
      pose.pose.orientation.w = 0.0;
      
      bool onInitPoint = false;
      
      ros::Rate sampling_hz(10);
      estimator_.enable();

      while(!onInitPoint)
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
              std::cout << station.header.frame_id << std::endl;
              tf2::Stamped<tf2::Transform> transf;
              tf2::fromMsg(station, transf);

              geometry_msgs::TransformStamped dpTFStamped;
              dpTFStamped = tf2::toMsg(transf);
              
              geometry_msgs::PoseStamped goalPose;
              pose.header = station.header;
              
              tf2::doTransform(pose, goalPose, dpTFStamped);
              goalPose.header = station.header;

              //Do a control step
              controller_.updateGoal(goalPose);
              controller_.enableControl();
              controller_.run();

              //If we are close enough to the goal, we are done (1cm error and )
              if(controller_.getDistanceError() < 0.05 && controller_.getOrientationError() < 0.05)
              {
                  ROS_INFO("Robot aligned with docking station");
                  controller_.disableControl();
                  estimator_.disable();
                  onInitPoint = true;
              }
          }
          sampling_hz.sleep();
      }
      
      controller_.disableControl();
      estimator_.disable();

      //Docking
      feedback.state = feedback.DOCKING;
      as_.publishFeedback(feedback);
      
      bool docked = false;
      
      sampling_hz.reset();
      
      pose.pose.position.x = 0.0;
      
      estimator_.enable();

      while(!docked)
      {
          //Cancel everything. Stop the robot
          if(as_.isPreemptRequested())
          {
              result.result = result.CHARGE_FAILED;
              as_.setAborted(result);
              controller_.disableControl();
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
              
              tf2::doTransform(pose, goalPose, dpTFStamped);
              goalPose.header = station.header;

              //Do a control step
              controller_.updateGoal(goalPose);
              controller_.enableControl();
              controller_.run();

              //If we are close enough to the goal, we are done (1cm error and 2.5deg error)
              //WE ALSO NEED TO CHECK IF THE ROBOT IS ACTUALLY CHARGING!

              if(controller_.getDistanceError() < 0.05 && controller_.getOrientationError() < 0.05)
              {
                  ROS_INFO("Robot docked!");
                  controller_.disableControl();
                  estimator_.disable();
                  docked = true;
              }
          }
        sampling_hz.sleep();
      }

      estimator_.disable();
      
      //Done - Charging
      feedback.state = feedback.CHARGING;
      as_.publishFeedback(feedback);
      result.result = result.CHARGE_SUCCESS;
      as_.setSucceeded(result);


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
{

}


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

