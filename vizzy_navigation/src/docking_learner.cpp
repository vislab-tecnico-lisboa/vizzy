/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <docking_learner.hpp>





void DockingLearner::addKeypointsInteractive(std::vector<FalkoBSCKeypoint> keypointslist, int color)
{
    if(color == GREEN)
    {
        keyPointMarker.controls.at(0).markers.at(0).color.r = 0;
        keyPointMarker.controls.at(0).markers.at(0).color.g = 1;
        keyPointMarker.controls.at(0).markers.at(0).color.b = 0;
    }else if (color == RED) {
        keyPointMarker.controls.at(0).markers.at(0).color.r = 1;
        keyPointMarker.controls.at(0).markers.at(0).color.g = 0;
        keyPointMarker.controls.at(0).markers.at(0).color.b = 0;
    }
    

    int i = 0;
    for(auto& k: keypointslist)
    {
        geometry_msgs::Pose keypoint_pose;
        tf2::Quaternion keypoint_orientation;
        keypoint_orientation.setRPY(0, 0, k.keypoint.orientation);
        keypoint_pose.orientation = tf2::toMsg(keypoint_orientation);
        keypoint_pose.position.x = k.keypoint.point.x();
        keypoint_pose.position.y = k.keypoint.point.y();
        keypoint_pose.position.z = 0;
        keyPointMarker.name = k.name;
        keyPointMarker.description = k.name;
        keyPointMarker.pose = keypoint_pose;
        markerServer->insert(keyPointMarker, boost::bind(&DockingLearner::markerFeedback, this, _1));

    }

}


void DockingLearner::markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {

        falkolib::Keypoint test;
        test.point = falkolib::Point2d(feedback->pose.position.x, feedback->pose.position.y);

        //Add clicked keypoint
        for(auto &k : keypointsDetected)
        {   
            
            if(k.keypoint.distance(test) < 0.01)
            {
                std::stringstream ss;
                ss << "Tracked"<<keypointsDatabase.size();
                auto tmp = k;
                tmp.name = ss.str();
                keypointsDatabase.push_back(tmp);
                ROS_INFO_STREAM("Added: " << tmp.keypoint.point);
                break;
            }
        }

        for(auto it = keypointsDatabase.begin(); it != keypointsDatabase.end(); )
        {
            if(it->keypoint.distance(test) < 0.02)
            {
                it = keypointsDatabase.erase(it);
                break;
            }
            else
                it++;

        }

        for(auto &k : keypointsDetected)
        {   
            if(k.name == feedback->marker_name)
            {
                std::stringstream ss;
                ss << "Tracked"<<keypointsDatabase.size();
                auto tmp = k;
                tmp.name = ss.str();
                keypointsDatabase.push_back(tmp);
                ROS_INFO_STREAM("Added: " << tmp.keypoint.point);
                break;
            }
        }

        markerServer->clear();
        markerServer->insert(saveMarker, boost::bind(&DockingLearner::saveMarkerFeedback, this, _1));
        addKeypointsInteractive(keypointsDetected, RED);
        addKeypointsInteractive(keypointsDatabase, GREEN);
        markerServer->applyChanges();

    }
}

void DockingLearner::saveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
        ROS_INFO("Saving Model");
        saveModel(_config_file, keypointsDatabase);
    }
}

void DockingLearner::laserCallback(const boost::shared_ptr<const sensor_msgs::LaserScan>& scan)
{

    markerServer->clear();
    markerServer->insert(saveMarker, boost::bind(&DockingLearner::saveMarkerFeedback, this, _1));

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

    //Extract the descriptors
    std::vector<falkolib::BSC> bscDesc;
    bsc_.compute(scan_falco, extractedKeypoints, bscDesc);

    
    //Create a list of detections that can be added to the database of KeyPoints from the click callback
    keypointsDetected.clear();

    for(int i = 0; i < extractedKeypoints.size(); i++)
    {   
        std::stringstream ss;
        ss << "Detected" << i;
        FalkoBSCKeypoint keypoint;
        keypoint.name = ss.str();
        keypoint.keypoint = extractedKeypoints.at(i);
        keypoint.descriptor = bscDesc.at(i);

        keypointsDetected.push_back(keypoint);
    }


    //Show saved points as red and detections as green. 
    //Check whether the detections are in the list of points to save and remove them if so
    for(auto &j : keypointsDatabase)
    {
        for(auto it = keypointsDetected.begin(); it != keypointsDetected.end();)
        {
            if(it->keypoint.orientation-j.keypoint.orientation < M_PI_4 &&
                it->keypoint.distance(j.keypoint) < 0.05)
            {
                it = keypointsDetected.erase(it);
            }
            else
            {
                it++;
            }
            
        }

    }

    //Interactive markers of each list to marker server
    addKeypointsInteractive(keypointsDetected, RED);
    addKeypointsInteractive(keypointsDatabase, GREEN);

    markerServer->applyChanges();

}

DockingLearner::DockingLearner(ros::NodeHandle nh) : nh_(nh), tfBuffer_(), tfListener_(tfBuffer_), bsc_(16, 8)
{

    std::stringstream ss;
    ss << ros::package::getPath("vizzy_navigation");
    ss << "/config/docking_model.yaml";
    _config_file = ss.str();

    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_filtered_rear", 1, &DockingLearner::laserCallback, this);
    debug_pub_ = nh_.advertise<sensor_msgs::LaserScan>("laser_filtered", 1);

    markerServer = new interactive_markers::InteractiveMarkerServer("KeyPoints");


    //Get a scan message to get the laser frame id - TODO
    std::string laser_frame_id;
    laser_frame_id = "nav_hokuyo_rear_laser_link";

    //A marker to save the gathered points
    saveMarker.header.frame_id = laser_frame_id;
    saveMarker.scale = 0.7;
    saveMarker.pose.position.x = 0;
    saveMarker.pose.position.y = 0;
    saveMarker.pose.position.z = 1;
    saveMarker.name = "Save Keypoints";
    saveMarker.description = "Save keypoints";

    visualization_msgs::Marker floppy;
    floppy.id = 1;
    floppy.ns = "SaveModel";
    floppy.type = visualization_msgs::Marker::CUBE;
    floppy.color.r = 0;
    floppy.color.g = 1;
    floppy.color.b = 0;
    floppy.color.a = 1;
    floppy.scale.x = 0.2;
    floppy.scale.y = 0.2;
    floppy.scale.z = 0.2;
    floppy.pose.orientation.x = 1;
    floppy.pose.orientation.y = 0;
    floppy.pose.orientation.z = 0;
    floppy.pose.orientation.w = 0;
    floppy.pose.position.x = 0;
    floppy.pose.position.y = 0;
    floppy.pose.position.z = 0.5;

    visualization_msgs::InteractiveMarkerControl clickSave;
    clickSave.always_visible = true;
    clickSave.markers.push_back(floppy);
    clickSave.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    saveMarker.controls.push_back(clickSave);

    //Keypoint markers
    visualization_msgs::Marker keyPoint;
    keyPoint.type = visualization_msgs::Marker::ARROW;
    keyPoint.scale.x = 0.15;
    keyPoint.scale.y = 0.05;
    keyPoint.scale.z = 0.05;
    keyPoint.pose.position.z = 0;
    keyPoint.color.r = 1.0;
    keyPoint.color.g = 0.0;
    keyPoint.color.b = 0.0;
    keyPoint.color.a = 1.0;


    //Markers to add / remove keypoints to be saved

    keyPointMarker.header.frame_id = laser_frame_id;
    keyPointMarker.scale = 0.2;
    keyPointMarker.name = "Keypoint: ";
    
    
    visualization_msgs::InteractiveMarkerControl clickKeypoint;
    clickKeypoint.always_visible = true;
    clickKeypoint.markers.push_back(keyPoint);
    clickKeypoint.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    keyPointMarker.controls.push_back(clickKeypoint);


    //Testing

    std::vector<falkolib::FALKO> keypoints;
    std::vector<falkolib::BSC> descriptors;
    loadModel(_config_file, keypoints, descriptors);
    
    std::cout << "DEBUG: " << descriptors.size() << std::endl;

    for(auto& desc : descriptors)
    {   
        std::cout << "----" << std::endl;
        std::cout << "descriptor.ringResolution: " << desc.ringResolution << std::endl;
        
        for(auto& i : desc.grid)
        {
            for(auto& j : i)
            {
                std::cout << int(j) << " ";
            }
            std::cout << std::endl;
        }
    }

}


DockingLearner::~DockingLearner()
{
    delete markerServer;
}
