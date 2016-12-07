#ifndef GAZESIMULATIONROSNODELET_H
#define GAZESIMULATIONROSNODELET_H
#include <nodelet/nodelet.h>
#include "../GazeSimulationRos.h"

namespace vizzy_control
{

class GazeSimulationRosNodelet: public nodelet::Nodelet
{

public:
    GazeSimulationRosNodelet(){}
    ~GazeSimulationRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<GazeSimulation> inst_;

};

}


#endif // GAZESIMULATIONROSNODELET_H

