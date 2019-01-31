/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <docking_estimator.hpp>

DockingEstimator::DockingEstimator(ros::NodeHandle nh) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_), bsc_(32, 16)
{

    std::stringstream ss;
    ss << ros::package::getPath("vizzy_navigation");
    ss << "/config/docking_model.yaml";
    _config_file = ss.str();

    std::cout << "_config_file: " << _config_file << std::endl;
    

    //Load the model
    loadModel(_config_file, model_keypoints_, model_descriptors_);

    std::cout << "model_keypoints_ size: " << model_keypoints_.size() << std::endl;
    std::cout << "model_descriptors_ size: " << model_descriptors_.size() << std::endl;


    //Subscribe the laser
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_filtered_rear", 1, &DockingEstimator::laserCallback, this);

    //Publish the position 
    docking_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/docking_pose", 1);

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

    matcherFALKOBSC_.setDistanceThreshold(1.0);
    matcherFALKOBSC_.setDescriptorThreshold(16);


    //Associate points with the model points
    std::vector<std::pair<int, int> > asso;
    matcherFALKOBSC_.match(extractedKeypoints, bscDesc, model_keypoints_, model_descriptors_, asso);

    //Compute the transform between points
    Eigen::Affine2d transformNN;

    
    falkolib::computeTransform(extractedKeypoints, model_keypoints_, asso, transformNN);


    std::cout << "Number of associations: " << asso.size() << std::endl;

	for (auto& match : asso) {
		if (match.second >= 0) {
			int i1 = match.first;
			int i2 = match.second;
			std::cout << "i1: " << i1 << "\ti2: " << i2 << "\t keypoints distance: " << (extractedKeypoints[i1].distance(model_keypoints_[i2])) << std::endl;
		}
	}


    //Convert Eigen to geometry_msgs

    double xx, xy, xz, yx, yy, yz, zx, zy, zz;
    double tx, ty, tz;

    auto m = transformNN.matrix();

    xx = m(0,0); xy = m(0,1); xz = 0;
    yx = m(1,0); yy = m(1,1); yz = 0;
    zx = 0; zy = 0; zz = 1;
    
    tx = m(0,2);
    ty = m(1,2);
    tz = 0;

    tf2::Matrix3x3 tm(xx, xy, xz, yx, yy, yz, zx, zy, zz);
    tf2::Vector3 translation(tx, ty, tz);
    tf2::Transform transform(tm, translation);

    geometry_msgs::Transform docking_poseTF;
    docking_poseTF = tf2::toMsg(transform);

    geometry_msgs::TransformStamped dpTFStamped;
    
    dpTFStamped.transform = docking_poseTF;
    
    //Transform the [0, 0, 0] point
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.w = 1;

    geometry_msgs::PoseStamped onLaser;

    tf2::doTransform(pose, onLaser, dpTFStamped);
    onLaser.header.frame_id = "nav_hokuyo_rear_laser_link";

    docking_pub_.publish(onLaser);

}