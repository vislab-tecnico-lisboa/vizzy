#ifndef DOCKING_LEARNER_HPP_
#define DOCKING_LEARNER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <falkolib/Feature/FALKOExtractor.h>
#include <falkolib/Feature/BSCExtractor.h>
#include <falkolib/Feature/BSC.h>
#include <falkolib/Matching/NNMatcher.h>
#include <exception>
#include <deque>

#include <falkolib/Feature/OCExtractor.h>
#include <falkolib/Feature/OC.h>


class DockingLearner
{
private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Subscriber laser_sub_;
    ros::Publisher points_pub_;
    ros::Publisher oc_pub_;
    ros::Publisher debug_pub_;

    std::deque<std::vector<double> > scan_buffer_;
    int scan_buffer_max = 1;
    int scan_buffer_size = 0;

    falkolib::FALKOExtractor fe_;
    falkolib::OCExtractor oce_;
    falkolib::BSCExtractor<falkolib::FALKO> bsc_;
    falkolib::NNMatcher<falkolib::FALKO, falkolib::BSC> matcherFALKOBSC_;

public:
    void laserCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan);
    DockingLearner(ros::NodeHandle nh);
    ~DockingLearner(){};

    
    

};


#endif