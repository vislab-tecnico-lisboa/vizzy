/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <docking_controller_ros.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "docking_controller");

  ros::NodeHandle nh;
  DockingControllerROS controllerROS(nh);

  ros::Rate sampling_hz(10);

  while(ros::ok())
  {
    ros::spinOnce();
    controllerROS.run();
    sampling_hz.sleep();
  }


  return 0;
}