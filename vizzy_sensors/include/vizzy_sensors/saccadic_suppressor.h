#ifndef SACCADIC_SUPPRESSOR_HPP
#define SACCADIC_SUPPRESSOR_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"
#include <dynamic_reconfigure/server.h>
#include <vizzy_sensors/SuppressionConfig.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>


/*Nodelet code heavily based on the image_proc cpp resize nodelet code*/


namespace vizzy_sensors
{

class SaccadicSuppression: public nodelet::Nodelet
{

protected:
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::NodeHandle> nPriv_;
    image_transport::Publisher pub_image_;
    ros::Publisher pub_info_;
    ros::Publisher pub_status_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_info_;

    //Checks the base velocity command
    ros::Subscriber sub_cmdvel_;
    geometry_msgs::Twist last_cmdvel_;
    std::mutex  angvel_mutex_;

    std::shared_ptr<image_transport::ImageTransport> it_, private_it_;
    std::mutex connect_mutex_; 

    // Dynamic reconfigure
    std::mutex config_mutex_;
    typedef vizzy_sensors::SuppressionConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    std::shared_ptr<ReconfigureServer> reconfigure_server_;
    Config config_;


public:
    SaccadicSuppression(){}
    ~SaccadicSuppression(){}
    virtual void onInit();

    void connectCb();

    void imageCb(const sensor_msgs::ImageConstPtr& image_msg);
    void infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);    

    void cmdVelCb(const geometry_msgs::Twist& cmdvel_msg);

    double getAngvel();

    void configCb(Config& config, uint32_t level);

};

}


#endif //SACCADIC_SUPPRESSOR_HPP