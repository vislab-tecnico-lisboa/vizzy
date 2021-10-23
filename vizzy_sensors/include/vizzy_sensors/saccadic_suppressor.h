#ifndef SACCADIC_SUPPRESSOR_HPP
#define SACCADIC_SUPPRESSOR_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <mutex>
#include <cv_bridge/cv_bridge.h>

/*Nodelet code heavily based on the image_proc cpp nodelet code*/


namespace vizzy_sensors
{

class SaccadicSuppression: public nodelet::Nodelet
{

protected:
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::NodeHandle> nPriv_;
    image_transport::Publisher pub_image_;
    ros::Publisher pub_info_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_info_;

    std::shared_ptr<image_transport::ImageTransport> it_, private_it_;
    std::mutex connect_mutex_; 

public:
    SaccadicSuppression(){}
    ~SaccadicSuppression(){}
    virtual void onInit();

    void connectCb();

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
    void infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);    


};

}


#endif //SACCADIC_SUPPRESSOR_HPP