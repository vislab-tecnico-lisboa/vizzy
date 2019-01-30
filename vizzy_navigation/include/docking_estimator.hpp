/*Copyright 2019, Joao Avelino, All rights reserved.*/


#ifndef DOCKING_LEARNER_HPP_
#define DOCKING_LEARNER_HPP_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <falko_database.hpp>
#include <ros/package.h>
#include <falkolib/Feature/FALKOExtractor.h>
#include <falkolib/Feature/BSCExtractor.h>
#include <falkolib/Feature/BSC.h>
#include <falkolib/Matching/NNMatcher.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class DockingEstimator
{

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Subscriber laser_sub_;
    ros::Publisher docking_pub_;

    std::vector<falkolib::FALKO> model_keypoints_;
    std::vector<falkolib::BSC> model_descriptors_;

    falkolib::FALKOExtractor fe_;
    falkolib::BSCExtractor<falkolib::FALKO> bsc_;
    falkolib::NNMatcher<falkolib::FALKO, falkolib::BSC> matcherFALKOBSC_;

    std::string _config_file;

    bool enabled_ = false;


public:
    DockingEstimator(ros::NodeHandle nh);
    ~DockingEstimator(){};
    void laserCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan);
    void enable(){enabled_ = true; };
    void disable() {enabled_ = false; };

};




#endif