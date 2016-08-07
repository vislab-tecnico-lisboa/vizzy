#ifndef GAZESIM_H
#define GAZESIM_H

#include "Gaze.h"

class GazeSimulation : public Gaze
{
public:
    typedef message_filters::sync_policies::ApproximateTime<control_msgs::JointControllerState, control_msgs::JointControllerState,control_msgs::JointControllerState, geometry_msgs::PointStamped> MySyncPolicy;
    boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > neck_pan_sub;
    boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > neck_tilt_sub;
    boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > eyes_tilt_sub;

    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;

    GazeSimulation(const std::string & name, const ros::NodeHandle & nh);
    bool moveHome();
    bool moveCartesian();
    void analysisCB(const control_msgs::JointControllerState::ConstPtr & neck_pan_msg,
                    const control_msgs::JointControllerState::ConstPtr & neck_tilt_msg,
                    const control_msgs::JointControllerState::ConstPtr & eyes_tilt_msg,
                    const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg);

};

#endif // GAZE_H
