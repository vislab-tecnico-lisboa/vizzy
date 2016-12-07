#include "GazeSimulationRos.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaze");
    ros::NodeHandle nh_;

    GazeSimulation gaze(ros::this_node::getName(),nh_);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}
