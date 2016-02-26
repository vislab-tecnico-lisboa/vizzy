#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav2d_operator/cmd.h>


class Twist2Nav2d
{

public:
    ros::NodeHandle nh;
    ros::NodeHandle nPriv;
    ros::Subscriber sub;
    ros::Publisher directCmd;
    ros::Publisher pub;
    int mode;

    Twist2Nav2d(ros::NodeHandle & nh_): nh(nh_), nPriv("~")
    {
        pub = nh.advertise<nav2d_operator::cmd>("nav2d_topic", 1);
        sub = nh.subscribe<geometry_msgs::Twist>("twist_topic", 1, &Twist2Nav2d::twistCallback, this);
        directCmd = nh.advertise<geometry_msgs::Twist>("cmd_topic", 1);
        nPriv.param<int>("mode", mode, 0);

        if(mode > 1)
            mode = 1;
    }


    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        nav2d_operator::cmd cmd;
        geometry_msgs::Twist direct_order;

        if((fabs(msg->linear.x) < 0.001) && (fabs(msg->linear.y) < 0.001) && (fabs(msg->linear.z) < 0.001) && (msg->angular.z != 0))
        { //If its a pure rotation, the robot wont colide so we don't need the Operator
        direct_order.angular.z = msg->angular.z*0.45;
        direct_order.angular.x = 0;
        direct_order.angular.y = 0;
        direct_order.linear.x = 0;
        direct_order.linear.y = 0;
        direct_order.linear.z = 0;

        if(fabs(direct_order.angular.z) < 0.01)
            direct_order.angular.z = 0;

        directCmd.publish(direct_order);

        }else{
        cmd.Velocity = msg->linear.x;
        cmd.Turn = -msg->angular.z;
        cmd.Mode = mode;

        //Deadzone
        if(fabs(cmd.Velocity) < 0.01)
            cmd.Velocity = 0;
        if(fabs(cmd.Turn) < 0.01)
            cmd.Turn = 0;


        pub.publish(cmd);
        }

    }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Twist2Nav2d");

  ros::NodeHandle n;

  Twist2Nav2d twist(n);

  ros::spin();
}
