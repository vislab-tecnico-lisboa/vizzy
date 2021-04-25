#include <mapless_nav/mapless_navigator.hpp>

using namespace std;


MaplessNavigator::MaplessNavigator(ros::NodeHandle &nh) : nh_(nh), state_(STOPPED), tfListener_(tfBuffer_),
                    tf2Filter_(poseSub_, tfBuffer_, "/base_footprint", 10, 0){

poseSub_.subscribe(nh_, "/mapless_goal", 1);
tf2Filter_.registerCallback( boost::bind(&MaplessNavigator::callback, this, _1) );
cmdPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 

robot_pose_.pose.position.x = 0;
robot_pose_.pose.position.y = 0;
robot_pose_.pose.position.z = 0;

robot_pose_.pose.orientation.x = 0;
robot_pose_.pose.orientation.y = 0;
robot_pose_.pose.orientation.z = 0;
robot_pose_.pose.orientation.w = 1;

}

MaplessNavigator::~MaplessNavigator(){


    std::cout << "Mapless navigator being destroyed. Sending zero velocity!" << std::endl;

    geometry_msgs::Twist vel;

    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    cmdPub_.publish(vel);

    std::cout << "Zero velocity sent to: " << cmdPub_.getTopic() << std::endl;

}

//Updaters the current goal
void MaplessNavigator::callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    try 
    {
      tfBuffer_.transform(*msg, current_goal_, "base_footprint");
      
      ROS_ERROR_STREAM("Current goal:" << current_goal_);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }

}


void MaplessNavigator::doControlBase()
{

    double x_rp = current_goal_.pose.position.x;
    double y_rp = current_goal_.pose.position.y;

    double rho = sqrt(x_rp*x_rp+y_rp*y_rp);

    if(rho < dist_tolerance_)
        return;
    
    double alpha = atan2(y_rp, x_rp);



}