#ifndef GAZEREAL_H
#define GAZEREAL_H

#include "Gaze.h"

class GazeReal : public Gaze
{
protected:
    ros::Publisher gazePublisher;
    ros::Subscriber fix_point_sub;
    std::string fixation_point_frame;
    geometry_msgs::PointStamped home_position_fixation_point;
public:
    GazeReal(const std::string & name, const ros::NodeHandle & nh);
    bool moveHome();
    bool moveCartesian();
    void analysisCB(const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg);
};

#endif // GAZEREAL_H
