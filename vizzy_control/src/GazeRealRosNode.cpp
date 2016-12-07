#include "GazeRealRos.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaze");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    GazeReal gaze(ros::this_node::getName(),nh_);
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}

