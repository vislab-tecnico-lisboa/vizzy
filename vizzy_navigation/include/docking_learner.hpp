/*Copyright 2019, Joao Avelino, All rights reserved.*/


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
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <falko_database>

#define RED 0
#define GREEN 1




class DockingLearner
{
private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Subscriber laser_sub_;
    ros::Publisher debug_pub_;

    //Marker stuff
    interactive_markers::InteractiveMarkerServer *markerServer;
    visualization_msgs::InteractiveMarker keyPointMarker;
    visualization_msgs::InteractiveMarker saveMarker;
    visualization_msgs::InteractiveMarker fixedLandmark;

    /*Databases of keypoints*/
    std::vector<FalkoBSCKeypoint> keypointsDetected;
    std::vector<FalkoBSCKeypoint> keypointsDatabase;

    std::deque<std::vector<double> > scan_buffer_;
    int scan_buffer_max = 1;
    int scan_buffer_size = 0;

    falkolib::FALKOExtractor fe_;
    falkolib::BSCExtractor<falkolib::FALKO> bsc_;
    falkolib::NNMatcher<falkolib::FALKO, falkolib::BSC> matcherFALKOBSC_;

public:
    void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void saveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void laserCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan);
    void addKeypointsInteractive(std::vector<FalkoBSCKeypoint> keypointslist, int color);
    DockingLearner(ros::NodeHandle nh);
    ~DockingLearner();

    
    

};


#endif