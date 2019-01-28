#include <docking_learner.hpp>

void DockingLearner::laserCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan)
{

    /*Atempt to filter scan signal to reduce noise...*/
    if(scan_buffer_size > scan_buffer_max)
    {
        scan_buffer_.pop_back();
        scan_buffer_.push_front(std::vector<double>(scan->ranges.begin(), scan->ranges.end()));
    }else{
        scan_buffer_.push_front(std::vector<double>(scan->ranges.begin(), scan->ranges.end()));
        scan_buffer_size++;
        return;
    }

    sensor_msgs::LaserScan debug_scan = *scan;

    std::vector<double> filtered_scan;

    /*Mean filter*/
    for(int i = 0; i < scan->ranges.size(); i++)
    {
        double val = 0;
        for(int j = 0; j < scan_buffer_.size(); j++)
        {
            val += scan_buffer_.at(j).at(i);
        }

        debug_scan.ranges.at(i) = val/scan_buffer_.size();
        filtered_scan.push_back(val/scan_buffer_.size());
    }

    
    debug_pub_.publish(debug_scan);
    

    //Convert sensor_msgs::LaserScan to falkolib::laserscan

    double angle_min;
    double fov;

    if(scan->angle_max-scan->angle_min < 0)
    {
        //The reading order is flipped?
        angle_min = (double) scan->angle_max;
        fov = (double) scan->angle_min-scan->angle_max;
    }else{
        //Normal case
        angle_min = (double) scan->angle_min;
        fov = (double) scan->angle_max-scan->angle_min;
        
    }

    falkolib::LaserScan scan_falco(angle_min, fov, scan->ranges.size());
    //std::vector<double> ranges(scan->ranges.begin(), scan->ranges.end());
    scan_falco.fromRanges(&filtered_scan[0]);

    //Extract the FALKO keypoints 
    std::vector<falkolib::FALKO> extractedKeypoints;
    fe_.extract(scan_falco, extractedKeypoints);

    //Extract the OC keypoints
    std::vector<falkolib::OC> extractedOCs;
    oce_.extract(scan_falco, extractedOCs);

    //Extract the descriptors
    std::vector<falkolib::BSC> bscDesc;
    bsc_.compute(scan_falco, extractedKeypoints, bscDesc);


    //Create a PoseStamped for each FALKO point 

    geometry_msgs::PoseArray msg;
    msg.header.frame_id=scan->header.frame_id;

    ROS_ERROR_STREAM("Keypoints extracted: " << extractedKeypoints.size());

    for(auto& k : extractedKeypoints)
    {
        geometry_msgs::Pose keypoint_pose;

        tf2::Quaternion keypoint_orientation;
        keypoint_orientation.setRPY(0, 0, k.orientation);

        keypoint_pose.orientation = tf2::toMsg(keypoint_orientation);

        keypoint_pose.position.x = k.point.x();
        keypoint_pose.position.y = k.point.y();
        keypoint_pose.position.z = 0;
      
        msg.poses.push_back(keypoint_pose);

    }

    points_pub_.publish(msg);

    //Create a PoseStamped for each OC point 

    geometry_msgs::PoseArray msg_oc;
    msg_oc.header.frame_id=scan->header.frame_id;

    ROS_ERROR_STREAM("Keypoints extracted: " << extractedOCs.size());

    for(auto& k : extractedOCs)
    {
        geometry_msgs::Pose keypoint_pose;

        tf2::Quaternion keypoint_orientation;
        keypoint_orientation.setRPY(0, 0, k.orientation);

        keypoint_pose.orientation = tf2::toMsg(keypoint_orientation);

        keypoint_pose.position.x = k.point.x();
        keypoint_pose.position.y = k.point.y();
        keypoint_pose.position.z = 0;
      
        msg_oc.poses.push_back(keypoint_pose);

    }

    oc_pub_.publish(msg_oc);


}

DockingLearner::DockingLearner(ros::NodeHandle nh) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_), bsc_(16, 8)
{

    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_filtered_rear", 1, &DockingLearner::laserCallback, this);
    points_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/falko", 1 );
    debug_pub_ = nh_.advertise<sensor_msgs::LaserScan>("laser_filtered", 1);
    oc_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/oc", 1 );

}


