#include "GazeSimulationRosNodelet.h"

namespace vizzy_control
{
    void GazeSimulationRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new GazeSimulation(ros::this_node::getName(), getPrivateNodeHandle()));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vizzy_control::GazeSimulationRosNodelet,nodelet::Nodelet)

