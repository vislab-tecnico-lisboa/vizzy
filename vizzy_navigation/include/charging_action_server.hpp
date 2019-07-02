#ifndef CHARGING_ACTION_SERVER_HPP_
#define CHARGING_ACTION_SERVER_HPP_


#include <docking_controller_ros.hpp>
#include <docking_estimator.hpp>
#include <actionlib/server/simple_action_server.h>
#include <vizzy_msgs/ChargeAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ChargingActionServer
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle n_priv;
    DockingControllerROS controller_;
    DockingEstimator estimator_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::string common_frame_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Time last_update_;
    actionlib::SimpleActionServer<vizzy_msgs::ChargeAction> as_;
    ros::Publisher cmd_pub_;

public:
    void controlToGoalPose(geometry_msgs::PoseStamped & pose, ros::Rate & sampling_hz, bool onDeadzone = false);
    void goalCallback();
    void preemptCB();
    void run();
    ChargingActionServer(ros::NodeHandle nh, std::string name);
    ~ChargingActionServer();
};




#endif //CHARGING_ACTION_SERVER_HPP_
