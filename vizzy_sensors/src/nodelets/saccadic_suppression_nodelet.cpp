/*Nodelet code heavily based on the image_proc cpp resize nodelet code*/


#include <vizzy_sensors/saccadic_suppressor.h>

namespace vizzy_sensors
{
    void SaccadicSuppression::onInit()
    {
        nh_.reset(new ros::NodeHandle(getNodeHandle()));
        nPriv_.reset(new ros::NodeHandle(getPrivateNodeHandle()));
        it_.reset(new image_transport::ImageTransport(*nh_));
        private_it_.reset(new image_transport::ImageTransport(*nPriv_));

        // Set up dynamic reconfigure
        reconfigure_server_.reset(new ReconfigureServer(*nPriv_));
        ReconfigureServer::CallbackType f = std::bind(&SaccadicSuppression::configCb, this, std::placeholders::_1, std::placeholders::_2);

        reconfigure_server_->setCallback(f);


        // Monitor whether anyone is subscribed to the output
        auto connect_cb = std::bind(&SaccadicSuppression::connectCb, this);

        // Make sure we don't enter connectCb() between advertising and assigning to
        // pub_XXX
        std::lock_guard<std::mutex> lock(connect_mutex_);

        pub_image_ = private_it_->advertise("image", 1, connect_cb, connect_cb);
        pub_info_ = nPriv_->advertise<sensor_msgs::CameraInfo>("camera_info", 1, connect_cb, connect_cb);
        pub_status_ = nPriv_->advertise<std_msgs::Bool>("suppressing", 1);

        time_stopsup_omega_ = ros::Time::now();
        time_stopsup_lap_ = ros::Time::now();

    }

    void SaccadicSuppression::configCb(Config& config, uint32_t level)
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = config;
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
            sub_cmdvel_ = nh_->subscribe("cmd_vel", 1, &SaccadicSuppression::cmdVelCb, this);
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

        //Return if image is older than 0.5ms
        if((ros::Time::now() - image_msg->header.stamp) > ros::Duration(0.5))
            return;

        Config config;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            config = config_;
        }

        cv_bridge::CvImageConstPtr cv_ptr;

        bool sup_lap = false; 
        bool sup_omega = false;
        bool sup_extra_lap = false; 
        bool sup_extra_omega = false;

        try{
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Code to analyze blur/glare in the image. If they are above a threshold
        // the robot will know that it is blind and discard the image

        cv::Mat src_gray;

        cvtColor(cv_ptr->image, src_gray, cv::COLOR_BGR2GRAY);

        cv::Mat laplacianed;
        cv::Laplacian(src_gray, laplacianed, CV_64F, 1, 1, 0, cv::BORDER_DEFAULT);
        cv::Scalar mn, stdev;
        cv::meanStdDev(laplacianed, mn, stdev);
        double variance = stdev.val[0]*stdev.val[0];

        if(variance < config.lap_var_thr)
        {
            sup_lap = true;
        }

        auto angvel = getAngvel();

        if(fabs(angvel) > config.base_omega_thr)
        {
            sup_omega = true;
        }

        
        //Activate extra suppression wait and return. We should probably check the headers
        //but with a queue size of 1 it should be enough
        if(last_sup_lap_ && !sup_lap)
        {
            sup_extra_lap = true;
            ros::Duration(config.extra_tlap/1000.0).sleep();
        }

        if(last_sup_omega_ && !sup_omega)
        {
            sup_extra_omega = true;
            ros::Duration(config.extra_tomega/1000.0).sleep();
        }

        last_sup_lap_ = sup_lap;
        last_sup_omega_ = sup_omega;

        std_msgs::Bool suppressing;

        if(!(sup_omega || sup_lap || sup_extra_lap || sup_extra_omega))
        {
            pub_image_.publish(image_msg);
            suppressing.data = false;
        }else{
            suppressing.data = true;
        }

        if(config.publish_suppress)
            pub_status_.publish(suppressing);

    }

    void SaccadicSuppression::cmdVelCb(const geometry_msgs::Twist& cmdvel_msg)
    {
        std::lock_guard<std::mutex> lock(angvel_mutex_);
        last_cmdvel_ = cmdvel_msg;
    }

    double SaccadicSuppression::getAngvel()
    {
        std::lock_guard<std::mutex> lock(angvel_mutex_);
        double angvel = last_cmdvel_.angular.z;
        return angvel;
    }

    
} // namespace vizzy_sensors

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_sensors::SaccadicSuppression, nodelet::Nodelet)
