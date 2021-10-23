#include <vizzy_sensors/saccadic_suppressor.h>

namespace vizzy_sensors
{
    void SaccadicSuppression::onInit()
    {
        nh_.reset(new ros::NodeHandle(getNodeHandle()));
        nPriv_.reset(new ros::NodeHandle(getPrivateNodeHandle()));
        it_.reset(new image_transport::ImageTransport(*nh_));
        private_it_.reset(new image_transport::ImageTransport(*nPriv_));


        // Monitor whether anyone is subscribed to the output
        auto connect_cb = std::bind(&SaccadicSuppression::connectCb, this);

        // Make sure we don't enter connectCb() between advertising and assigning to
        // pub_XXX
        std::lock_guard<std::mutex> lock(connect_mutex_);

        pub_image_ = private_it_->advertise("image", 1, connect_cb, connect_cb);
        pub_info_ = nPriv_->advertise<sensor_msgs::CameraInfo>("camera_info", 1, connect_cb, connect_cb);

    }

    void SaccadicSuppression::connectCb()
    {
        std::lock_guard<std::mutex> lock(connect_mutex_);
        if (pub_image_.getNumSubscribers() == 0)
        {
            sub_image_.shutdown();
        }else if(!sub_image_)
        {
            sub_image_ = it_->subscribe("image", 1, &SaccadicSuppression::imageCb, this);
        }
        if (pub_info_.getNumSubscribers() == 0)
        {
            sub_info_.shutdown();
        }
        else if(!sub_info_)
        {
            sub_info_ = nh_->subscribe<sensor_msgs::CameraInfo>("camera_info", 1, &SaccadicSuppression::infoCb, this);
        }
    }

    void SaccadicSuppression::infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        pub_info_.publish(info_msg);
    }

    void SaccadicSuppression::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;

        try{
            cv_ptr = cv_bridge::toCvShare(image_msg);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Code to analyze blur/glare in the image. If they are above a threshold
        // the robot will know that it is blind and discard the image

        pub_image_.publish(image_msg);

    }
    
} // namespace vizzy_sensors

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_sensors::SaccadicSuppression, nodelet::Nodelet)
