#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"


class PolygonStampedToPolygon
{
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::NodeHandle n;

    /**
     * This tutorial demonstrates simple receipt of messages over the ROS system.
     */
    void polygonStampedCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
    {
        pub.publish(msg->polygon);
        sub.shutdown();
    }

public:
    PolygonStampedToPolygon(const ros::NodeHandle & n_): n(n_)
    {
        sub=n.subscribe("polygon_stamped_in", 100, &PolygonStampedToPolygon::polygonStampedCallback,this);
        pub=n.advertise<geometry_msgs::Polygon>("polygon_out", 1,true);
    }
};



int main(int argc, char **argv)
{

    ros::init(argc, argv, "polygon_stamped_to_polygon");
    ros::NodeHandle n;

    PolygonStampedToPolygon polygon_stamped_to_polygon(n);
    /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
    ros::spin();

    return 0;
}
