/*Copyright 2019, Joao Avelino, All rights reserved.*/

/*This node estimates the position of the docking station*/

#include <docking_estimator.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "docking_estimator");

  ros::NodeHandle nh;
  DockingEstimator dockingLearner(nh);

  ros::Rate sampling_hz(10);

  //dockingLearner.enable();

  while(ros::ok())
  {
    ros::spinOnce();
    sampling_hz.sleep();
  }


  return 0;
}