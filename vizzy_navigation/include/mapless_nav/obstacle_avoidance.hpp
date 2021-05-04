#ifndef MAPLESS_OBSAVOIDANCE_HPP_
#define MAPLESS_OBSAVOIDANCE_HPP_

/*Code from https://github.com/skasperski/navigation_2d/blob/master/nav2d_operator/src/RobotOperator.cpp
with minor adaptations to this use case.*/
/*Original code from: Sebastian Kasperski: https://github.com/skasperski*/

#define ROUTE_TOPIC   "route"
#define PLAN_TOPIC    "desired"
#define LUT_RESOLUTION 100
#define PI 3.14159265


#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GridCells.h>
#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

class RobotOperator
{
public:
	// Default Constructor & Destructor
	RobotOperator(ros::NodeHandle &nh, ros::NodeHandle &nPriv);
	~RobotOperator();
	
	// Public Methods
	/**
	 * @brief Originally a callback function to receive move commands. Now converts a twist
	 * message to nav2d
	 * @param cmd_vel Desired velocity
	 */
	void receiveCommand(geometry_msgs::Twist &msg);

	/**
	 * @brief Generates Twist-Message that considers abstacles
	 * This is the Operator's core function and should be called periodically
	 */
	geometry_msgs::Twist computeVelocity();

	double mMaxVelocity;
	
	bool mPublishRoute;
	double mMaxFreeSpace;
	double mSafetyDecay;
	int mSafetyWeight;
	int mConformanceWeight;
	int mContinueWeight;
	int mEscapeWeight;
	int mDriveMode;

	double mMaxLinearVelocity;
	double mMaxAngularVelocity;

	bool rStuck = false;

private:
	// Internal Methods
	/**
	 * @brief Calculates the distance the robot can move following the given trajectory
	 * @param cloud PointCloud defining a trajectory
	 * @return Nmber of free cells
	 */
	int calculateFreeSpace(sensor_msgs::PointCloud* cloud);

	/**
	 * @brief Calculate the action value of a given command
	 * @param direction How to move the robot
	 * @param velocity Only used to distinguish forward and backward motion
	 * @param debug Publish result of evaluation functions on debug topic
	 * @return Weighted sum of all evaluation functions
	 * The given action is rated by 4 different evaluation functions:
	 * Free Space: How far can the robot move following this command
	 * Safety: How close will it get near obstacles
	 * Conformance: How good does it follow the commanded direction
	 * Continuity: (experimental!) How does it conform with the last issued command
	 */
	double evaluateAction(double direction, double velocity, bool debug = false);

	/**
	 * @brief Evaluates all possible directions with a fixed resolution
	 * @return Best evaluated direction
	 */
	double findBestDirection();

	/**
	 * @brief Initializes look-up tables
	 * This calculates the trajectories of all possible direction commands.
	 * Must be called once before the Operator is used
	 */
	void initTrajTable();
	
	/**
	 * @brief Get the trajectory defined by the given movement command
	 * @param direction How to move the robot
	 * @param velocity Only used to distinguish forward and backward motion
	 * @return A pointer to the PointCloud defined in the robot coordinate frame
	 */
	inline sensor_msgs::PointCloud* getPointCloud(double direction, double velocity);

	ros::NodeHandle nh;
	ros::NodeHandle nPriv;

	// Internal Storage
	costmap_2d::Costmap2DROS* mLocalMap;
	costmap_2d::Costmap2D* mCostmap;
	double mRasterSize;
	
	ros::Publisher mTrajectoryPublisher;
	ros::Publisher mPlanPublisher;
	ros::Publisher mCostPublisher;

	tf::TransformListener mTfListener;
	tf2_ros::Buffer mTf2Buffer;
	tf2_ros::TransformListener mTf2Listener;
	
	double mDesiredVelocity;
	double mDesiredDirection;
	double mCurrentVelocity;
	double mCurrentDirection;

	
	sensor_msgs::PointCloud* mTrajTable[(LUT_RESOLUTION * 4) + 2];
	

	std::string mOdometryFrame;
	std::string mRobotFrame;
	
	unsigned int mRecoverySteps;

};



#endif