#include <docking_controller_ros.hpp>

void callback(vizzy_navigation::DockingConfig &config, uint32_t level) {


  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "docking_controller");

  ros::NodeHandle nh;
  DockingControllerROS controllerROS(nh);
  
  
  ros::spin();


  return 0;
}