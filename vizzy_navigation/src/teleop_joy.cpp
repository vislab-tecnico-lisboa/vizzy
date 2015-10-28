// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
// %EndTag(INCLUDE)%
// %Tag(CLASSDEF)%
class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};
// %EndTag(CLASSDEF)%
// %Tag(PARAMS)%
TeleopJoy::TeleopJoy(): priv_nh_("~"),
  linear_(1),
  angular_(2)
{

  priv_nh_.param("axis_linear", linear_, linear_);
  priv_nh_.param("axis_angular", angular_, angular_);
  priv_nh_.param("scale_angular", a_scale_, a_scale_);
  priv_nh_.param("scale_linear", l_scale_, l_scale_);
// %EndTag(PARAMS)%
// %Tag(PUB)%
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
// %EndTag(PUB)%
// %Tag(SUB)%
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);
// %EndTag(SUB)%
}
// %Tag(CALLBACK)%
void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;

  vel.linear.x = l_scale_*joy->axes[1];
  vel.angular.z = a_scale_*joy->axes[2];
  vel_pub_.publish(vel);
}
// %EndTag(CALLBACK)%
// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_turtle;

  ros::spin();
}
// %EndTag(MAIN)%
// %EndTag(FULL)%

