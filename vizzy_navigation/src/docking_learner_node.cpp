/*Copyright 2019, Joao Avelino, All rights reserved.*/

/*This node creates a model of meaningful 2D LIDAR features for the docking station*/

#include <docking_learner.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "docking_learner");

  ros::NodeHandle nh;
  DockingLearner dockingLearner(nh);

  ros::Rate sampling_hz(10);

  while(ros::ok())
  {
    ros::spinOnce();
    sampling_hz.sleep();
  }


  return 0;
}