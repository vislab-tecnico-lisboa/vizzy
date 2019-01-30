/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <docking_estimator.hpp>

DockingEstimator::DockingEstimator(ros::NodeHandle nh) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_), bsc_(16, 8)
{

    std::stringstream ss;
    ss << ros::package::getPath("vizzy_navigation");
    ss << "/config/docking_model.yaml";
    _config_file = ss.str();

    //Load the model
    loadModel(_config_file, model_keypoints_, model_descriptors_);

}

void DockingEstimator::laserCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan)
{
    
    if(!enabled_)
        return;
    

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
    std::vector<double> ranges(scan->ranges.begin(), scan->ranges.end());
    scan_falco.fromRanges(&ranges[0]);

    //Extract the keypoints
    std::vector<falkolib::FALKO> extractedKeypoints;
    fe_.extract(scan_falco, extractedKeypoints);


    //Extract the descriptors
    std::vector<falkolib::BSC> bscDesc;
    bsc_.compute(scan_falco, extractedKeypoints, bscDesc);

    matcherFALKOBSC_.setDistanceThreshold(0.1);
    matcherFALKOBSC_.setDescriptorThreshold(15);

    //Associate points with the model points
    std::vector<std::pair<int, int> > asso;
    matcherFALKOBSC_.match(extractedKeypoints, bscDesc, model_keypoints_, model_descriptors_, asso);

    //Compute the transform between points
    Eigen::Affine2d transformNN;
    falkolib::computeTransform(extractedKeypoints, model_keypoints_, asso, transformNN);

    std::cout << transformNN.inverse().matrix() << std::endl;


}