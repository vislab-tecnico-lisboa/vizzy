/*Copyright 2019, Joao Avelino, Rui P. Figueiredo, All rights reserved.*/


#ifndef DOCKING_LEARNER_HPP_
#define DOCKING_LEARNER_HPP_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pattern_pose_estimation.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <laser_geometry/laser_geometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
class DockingEstimator
{
	private:
	    ros::NodeHandle nh_;
	    ros::NodeHandle n_priv;
	    tf2_ros::Buffer tfBuffer_;
	    tf2_ros::TransformListener tfListener_;
	    ros::Subscriber laser_sub_;
	    ros::Publisher docking_pub_;
	    ros::Publisher model_pub_;
		geometry_msgs::PoseStamped onLaser;
		std::deque<double> yaw_filter;
		std::deque<double> x_filter;
		std::deque<double> y_filter;
		std::deque<double> z_filter;

	    std::string _config_file;
	    bool enabled_ = false;
		bool ready_ = false;
	    std::shared_ptr<PatternPoseEstimation> pattern_pose_estimation;

	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals;

	public:
	    geometry_msgs::PoseStamped getPatternPose();
	    DockingEstimator(ros::NodeHandle nh);
	    ~DockingEstimator(){};
	    void laserCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan);
	    bool isReady(){return ready_;};
	    void enable(){enabled_ = true; };
	    void disable() {enabled_ = false; };
            double findMedian(std::deque<double> a);
};
#endif
