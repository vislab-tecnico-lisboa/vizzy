#include <docking_controller_ros.hpp>


/*Controller initialized with default parameters*/
DockingControllerROS::DockingControllerROS(ros::NodeHandle nh) : nh_(nh), controller(3.0, 8.0, -1.5)
{
  
  f = boost::bind(&DockingControllerROS::dynamic_rec_callback, this, _1, _2);
  server.setCallback(f);

}

DockingControllerROS::~DockingControllerROS(){}

void DockingControllerROS::dynamic_rec_callback(vizzy_navigation::DockingConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: k_ro: %f,  k_alpha: %f, k_beta: %f", 
            config.k_ro, config.k_alpha, config.k_beta);

  controller.updateGains(config.k_ro, config.k_alpha, config.k_beta);
}

void DockingControllerROS::run()
{
    ros::spinOnce();

    /*Compute ro, alpha and beta*/

    /****************************/
    
    float ro = 0;
    float alpha = 0;
    float beta = 0;

    controller.computeControlSignal(ro, alpha, beta);

}