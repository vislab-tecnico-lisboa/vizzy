/*Copyright 2019, Joao Avelino, Rui P. Figueiredo, All rights reserved.*/

#include <docking_estimator.hpp>

DockingEstimator::DockingEstimator(ros::NodeHandle nh) : 
    nh_(nh), 
    n_priv("~"), 
    tfBuffer_(), 
    tfListener_(tfBuffer_)
{
    double tran_thresh;
    double rot_thresh;
    double fitting_score_thresh;
    double discretization_step;

    std::string model_file;

    n_priv.param("tran_thresh",tran_thresh,0.05); // in meters
    n_priv.param("rot_thresh",rot_thresh,30.0); // in degrees
    n_priv.param("fitting_score_thresh",fitting_score_thresh,0.01); // in degrees
    n_priv.param("discretization_step",discretization_step,0.01); // in degrees

    n_priv.param<std::string>("model_file",model_file,"file"); // in degrees

    ROS_INFO_STREAM("tran_thres:"<<tran_thresh);
    ROS_INFO_STREAM("rot_thresh:"<<rot_thresh);
    ROS_INFO_STREAM("fitting_score_thresh:"<<fitting_score_thresh);
    ROS_INFO_STREAM("discretization_step:"<<discretization_step);

    // Train PPF registration model
    pattern_pose_estimation=std::shared_ptr<PatternPoseEstimation>(new PatternPoseEstimation(
        rot_thresh,
        tran_thresh,
        fitting_score_thresh,
        discretization_step,
        model_file));

    //Subscribe the laser
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_filtered_rear", 1, &DockingEstimator::laserCallback, this);

    //Publish the position 
    docking_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/docking_pose", 1);

    //Publish the model
    model_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> > ("model_point_cloud", 1);
}

void DockingEstimator::laserCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan)
{

    if(!enabled_)
        return;
    
    

    // Convert to pcl
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan, cloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new  pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *cloud_pcl);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals=pattern_pose_estimation->getPointNormal(cloud_pcl);

    //Compute the transform between points
    Eigen::Affine3d transformNN;
    try
    {
    	transformNN=pattern_pose_estimation->detect(cloud_normals);
    }
    catch(std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return;
    }

    
    tf::poseEigenToMsg (transformNN, onLaser.pose);

    onLaser.header = scan->header;

    Eigen::Affine3d affine_tf;
    Eigen::Matrix3d Tm;
    affine_tf.matrix()=mat_filtered;

    tf::poseEigenToMsg(affine_tf, onLaser.pose);
    onLaser.header = scan->header;
    pattern_pose_estimation->cloud_output_subsampled->header.frame_id=scan->header.frame_id;
    pcl_conversions::toPCL(scan->header, pattern_pose_estimation->cloud_output_subsampled->header);
    model_pub_.publish(pattern_pose_estimation->cloud_output_subsampled);
    docking_pub_.publish(onLaser);

    ready_ = true;
}

geometry_msgs::PoseStamped DockingEstimator::getPatternPose()
{
    return onLaser;
}

// Function for calculating median 
double DockingEstimator::findMedian(std::deque<double> & a) 
{ 
    // First we sort the array 
    if(a.size()>1.0)
  	    std::sort(a.begin(), a.end()); 

    // check for even case 
    if (a.size() % 2 != 0) 
       return (double) a.at(floor(a.size()/2.0)); 

    return (double) (a.at(floor((a.size()-1)/2.0)) + a.at(floor(a.size()/2.0)))/2.0; 
} 

