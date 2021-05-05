#include <mapless_nav/obstacle_avoidance.hpp>
#include <math.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

RobotOperator::RobotOperator(ros::NodeHandle &nh, ros::NodeHandle &nPriv) : nh(nh), nPriv(nPriv), mTf2Listener(mTf2Buffer)
{
	// Create the local costmap
	#if ROS_VERSION_MINIMUM(1, 14, 5) // If melodic or newer, use costmap2d with TF2
		mLocalMap = new costmap_2d::Costmap2DROS("local_map", mTf2Buffer);
	#else //Otherwise use costmap2d with TF
		mLocalMap = new costmap_2d::Costmap2DROS("local_map", mTfListener);
	#endif
	mRasterSize = mLocalMap->getCostmap()->getResolution();
	
	// Publish / subscribe to ROS topics
	nPriv.param("robot_frame", mRobotFrame, std::string("base_footprint"));
	nPriv.param("odometry_frame", mOdometryFrame, std::string("odom"));
	mCostPublisher = nPriv.advertise<geometry_msgs::Vector3>("costs", 1);
	
	nPriv.param("publish_route", mPublishRoute, false);
	if(mPublishRoute)
	{
		ROS_INFO("Will publish desired direction on '%s' and control direction on '%s'.", ROUTE_TOPIC, PLAN_TOPIC);
		mTrajectoryPublisher = nPriv.advertise<nav_msgs::GridCells>(ROUTE_TOPIC, 1);
		mPlanPublisher = nPriv.advertise<nav_msgs::GridCells>(PLAN_TOPIC, 1);
	}


	// Initialize the lookup table for the driving directions
	ROS_INFO("Initializing LUT...");
	initTrajTable();
	ROS_INFO("...done!");
	
	// Set internal parameters
	mDesiredDirection = 0;
	mDesiredVelocity = 0;
	mCurrentDirection = 0;
	mCurrentVelocity = 0;
	mDriveMode = 0;
	mRecoverySteps = 0;
}

RobotOperator::~RobotOperator()
{
	for(int i = 0; i < LUT_RESOLUTION; i++)
	{
		delete mTrajTable[i];
	}
}

void RobotOperator::initTrajTable()
{
	for(int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
	{
		mTrajTable[i] = NULL;
	}	
	for(int i = 1; i < LUT_RESOLUTION; i++)
	{
		double tw = -PI * i / LUT_RESOLUTION;
		double tx = cos(tw) + 1;
		double ty = -sin(tw);
		double tr = ((tx*tx)+(ty*ty))/(ty+ty);
		std::vector<geometry_msgs::Point32> points;
		double alpha = 0;
		while(alpha < PI)
		{
			double x = tr * sin(alpha);
			double y = tr * (1.0 - cos(alpha));
			geometry_msgs::Point32 p;
			p.x = x;
			p.y = y;
			p.z = 0;
			points.push_back(p);
			alpha += mRasterSize / tr;
		}
		// Add the PointCloud to the LUT
		// Circle in forward-left direction
		sensor_msgs::PointCloud* flcloud = new sensor_msgs::PointCloud();
		flcloud->header.stamp = ros::Time(0);
		flcloud->header.frame_id = mRobotFrame;
		flcloud->points.resize(points.size());
		
		// Circle in forward-right direction
		sensor_msgs::PointCloud* frcloud = new sensor_msgs::PointCloud();
		frcloud->header.stamp = ros::Time(0);
		frcloud->header.frame_id = mRobotFrame;
		frcloud->points.resize(points.size());
		
		// Circle in backward-left direction
		sensor_msgs::PointCloud* blcloud = new sensor_msgs::PointCloud();
		blcloud->header.stamp = ros::Time(0);
		blcloud->header.frame_id = mRobotFrame;
		blcloud->points.resize(points.size());
		
		// Circle in backward-right direction
		sensor_msgs::PointCloud* brcloud = new sensor_msgs::PointCloud();
		brcloud->header.stamp = ros::Time(0);
		brcloud->header.frame_id = mRobotFrame;
		brcloud->points.resize(points.size());
		
		for(unsigned int j = 0; j < points.size(); j++)
		{
			flcloud->points[j] = points[j];
			frcloud->points[j] = points[j];
			blcloud->points[j] = points[j];
			brcloud->points[j] = points[j];
			
			frcloud->points[j].y *= -1;
			blcloud->points[j].x *= -1;
			brcloud->points[j].x *= -1;
			brcloud->points[j].y *= -1;
		}
		mTrajTable[LUT_RESOLUTION - i] = flcloud;
		mTrajTable[LUT_RESOLUTION + i] = frcloud;
		mTrajTable[(3 * LUT_RESOLUTION + 1) - i] = blcloud;
		mTrajTable[(3 * LUT_RESOLUTION + 1) + i] = brcloud;
	}
	
	// Add First and Last LUT-element
	geometry_msgs::Point32 p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	
	sensor_msgs::PointCloud* turn = new sensor_msgs::PointCloud();
	turn->header.stamp = ros::Time(0);
	turn->header.frame_id = mRobotFrame;
	turn->points.resize(1);
	turn->points[0] = p;
	
	int straight_len = 5.0 / mRasterSize;
	
	sensor_msgs::PointCloud* fscloud = new sensor_msgs::PointCloud();
	fscloud->header.stamp = ros::Time(0);
	fscloud->header.frame_id = mRobotFrame;
	fscloud->points.resize(straight_len);
	
	sensor_msgs::PointCloud* bscloud = new sensor_msgs::PointCloud();
	bscloud->header.stamp = ros::Time(0);
	bscloud->header.frame_id = mRobotFrame;
	bscloud->points.resize(straight_len);
	
	for(int i = 0; i < straight_len; i++)
	{
		fscloud->points[i] = p;
		bscloud->points[i] = p;
		bscloud->points[i].x *= -1;
		p.x += mRasterSize;
	}
	
	mTrajTable[LUT_RESOLUTION] = fscloud;
	mTrajTable[LUT_RESOLUTION*3 + 1] = bscloud;
	
	mTrajTable[0] = turn;
	mTrajTable[LUT_RESOLUTION*2] = turn;
	mTrajTable[LUT_RESOLUTION*2 + 1] = turn;
	mTrajTable[LUT_RESOLUTION*4 + 1] = turn;
	
	for(int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
	{
		if(!mTrajTable[i])
		{
			ROS_ERROR("Table entry %d has not been initialized!", i);
		}
	}	
}

void RobotOperator::receiveCommand(geometry_msgs::Twist &msg)
{

	/*Convert from nav2d operator message with Velocity and Turn to Twist*/
	double& ang_in = msg.angular.z;
	double& lin_in = msg.linear.x;

	double velocity;
	double turn;

    if(ang_in == 0.0)
	{
        velocity = lin_in;
        turn = ang_in;
	}
    else if(lin_in == 0.0 && ang_in != 0.0)
	{
        turn = -1.0*sgn(ang_in);
        velocity = ang_in/(-1.0*turn);
	}
    else
	{
        double r = -lin_in/ang_in;
        double abs_r = fabs(r);
        double aux = 1.0+(1.0/abs_r);
        velocity = lin_in*aux;

		double arc = 2*atan2(-ang_in, lin_in);
		if(arc > M_PI)
		{
			int n = floor(arc/M_PI);
			arc -= (n+1)*M_PI;
		}else if(arc < -M_PI)
		{
			int n = floor(-arc/M_PI);
			arc += (n+1)*M_PI;
		}
        turn = arc/M_PI;
	}

	
	if(turn < -1 || turn > 1)
	{
		// The given direction is invalid.
		// Something is going wrong, so better stop the robot:
		mDesiredDirection = 0;
		mDesiredVelocity = 0;
		mCurrentDirection = 0;
		mCurrentVelocity = 0;
		ROS_ERROR_THROTTLE(5, "Invalid turn direction!");
        ROS_ERROR_STREAM_THROTTLE(5, "Received velocity: " << velocity);
        ROS_ERROR_STREAM_THROTTLE(5, "Received turn: " << turn);

		ROS_ERROR_STREAM_THROTTLE(5, msg);
		return;
	}

	mDesiredDirection = turn;
	mDesiredVelocity = velocity;

	return;

}

geometry_msgs::Twist RobotOperator::computeVelocity()
{
	geometry_msgs::Twist controlMsg;

	// 1. Get a copy of the costmap to work on.
	mCostmap = mLocalMap->getCostmap();
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));
	double bestDirection, d;

	int localDriveMode;

	if(mode_switched)
		localDriveMode = 1-mDriveMode;
	else
		localDriveMode = mDriveMode;
	
	// 2. Set velocity and direction depending on drive mode
	switch(localDriveMode)
	{
	case 0:
		bestDirection = findBestDirection();
		d = bestDirection - mCurrentDirection;
		if(d < -0.2) d = -0.2;
		if(d > 0.2) d = 0.2;
		mCurrentDirection += d;
		mCurrentVelocity = mDesiredVelocity;
		break;
	case 1:
		mCurrentDirection = mDesiredDirection;
		mCurrentVelocity = mDesiredVelocity;
		break;
	default:
		ROS_ERROR("Invalid drive mode!");
		mCurrentVelocity = 0.0;
	}
	
	// Create some Debug-Info
	evaluateAction(mCurrentDirection, mCurrentVelocity, true);
	
	sensor_msgs::PointCloud* originalCloud = getPointCloud(mCurrentDirection, mDesiredVelocity);
	sensor_msgs::PointCloud transformedCloud;

	try
	{
		mTfListener.transformPointCloud(mOdometryFrame,*originalCloud,transformedCloud);
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		controlMsg.angular.x = 0;
		controlMsg.angular.y = 0;
		controlMsg.angular.z = 0;
		controlMsg.linear.x = 0;
		controlMsg.linear.y = 0;
		controlMsg.linear.z = 0;

		return controlMsg;
	}
	
	// Determine maximum linear velocity
	int freeCells = calculateFreeSpace(&transformedCloud);
	double freeSpace = mRasterSize * freeCells;

	double safeVelocity = (freeSpace / mMaxFreeSpace) + 0.05;
	if(freeCells == transformedCloud.points.size() && safeVelocity < 0.5)
		safeVelocity = 0.5;
		
	if(freeSpace < 0.3 && freeCells < transformedCloud.points.size())
		safeVelocity = 0;

	if(safeVelocity > mMaxVelocity)
		safeVelocity = mMaxVelocity;

	// Check whether the robot is stuck
	if(mRecoverySteps > 0) mRecoverySteps--;

	//Switch mode if recovering
	

	if(safeVelocity < 0.1)
	{
		if(localDriveMode == 0)
		{
			mRecoverySteps = 30; // Recover for 3 seconds
			ROS_WARN_THROTTLE(1, "Robot is stuck! Trying to recover...");
		}else
		{
			mCurrentVelocity = 0;
			ROS_WARN_THROTTLE(1, "Robot cannot move further in this direction!");
			ROS_WARN_THROTTLE(1, "STOPING ROBOT!");
			rStuck = true;
			if(mode_switched)
				mode_switched = !mode_switched;
		}
	}

	// Publish route via ROS (mainly for debugging)
	if(mPublishRoute)
	{
		nav_msgs::GridCells route_msg;
		route_msg.header.frame_id = mOdometryFrame;
		route_msg.header.stamp = ros::Time::now();
	
		route_msg.cell_width = mCostmap->getResolution();
		route_msg.cell_height = mCostmap->getResolution();
	
		route_msg.cells.resize(freeCells);
		for(int i = 0; i < freeCells; i++)
		{
			route_msg.cells[i].x = transformedCloud.points[i].x;
			route_msg.cells[i].y = transformedCloud.points[i].y;
			route_msg.cells[i].z = transformedCloud.points[i].z;
		}
		mTrajectoryPublisher.publish(route_msg);
	
		// Publish plan via ROS (mainly for debugging)
		sensor_msgs::PointCloud* originalPlanCloud = getPointCloud(mDesiredDirection, mDesiredVelocity);
		sensor_msgs::PointCloud transformedPlanCloud;

		try
		{
			mTfListener.transformPointCloud(mOdometryFrame,*originalCloud,transformedCloud);
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			controlMsg.angular.x = 0;
			controlMsg.angular.y = 0;
			controlMsg.angular.z = 0;
			controlMsg.linear.x = 0;
			controlMsg.linear.y = 0;
			controlMsg.linear.z = 0;

			return controlMsg;
		}
		nav_msgs::GridCells plan_msg;
		plan_msg.header = route_msg.header;
	
		plan_msg.cell_width = mCostmap->getResolution();
		plan_msg.cell_height = mCostmap->getResolution();
	
		int freeSpacePlan = calculateFreeSpace(&transformedPlanCloud);
		plan_msg.cells.resize(freeSpacePlan);
		for(int i = 0; i < freeSpacePlan; i++)
		{
			plan_msg.cells[i].x = transformedPlanCloud.points[i].x;
			plan_msg.cells[i].y = transformedPlanCloud.points[i].y;
			plan_msg.cells[i].z = transformedPlanCloud.points[i].z;
		}
		mPlanPublisher.publish(plan_msg);
	}
	
	double velocity = mCurrentVelocity;
	if(mCurrentDirection == 0)
	{
		if(velocity > safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safeVelocity);
			velocity = safeVelocity;
		}else if(velocity < -safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safeVelocity);
			velocity = -safeVelocity;
		}
		controlMsg.linear.x = velocity;
		controlMsg.angular.z = 0;
	}else if(mCurrentDirection == -1 || mCurrentDirection == 1)
	{
		controlMsg.linear.x = 0;
		controlMsg.angular.z = -1.0 * mCurrentDirection * velocity;
	}else
	{
		double x = sin(mCurrentDirection * PI);
		double y = (cos(mCurrentDirection * PI) + 1);
		double r = ((x*x) + (y*y)) / (2*x);
		double abs_r = (r > 0) ? r : -r;
		velocity /= (1 + (1.0/abs_r));
		if(velocity > safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safeVelocity);
			velocity = safeVelocity;
		}else if(velocity < -safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safeVelocity);
			velocity = -safeVelocity;
		}
		
		controlMsg.linear.x = velocity;
		controlMsg.angular.z = -1.0 / r * controlMsg.linear.x;
	}
	return controlMsg;
}

int RobotOperator::calculateFreeSpace(sensor_msgs::PointCloud* cloud)
{	
	unsigned int mx, my;
	int length = cloud->points.size();
	int freeSpace = 0;
	for(int i = 0; i < length; i++)
	{
		if(mCostmap->worldToMap(cloud->points[i].x, cloud->points[i].y, mx, my))
		{
			if(mCostmap->getCost(mx,my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				freeSpace++;
			}else
			{
				break;
			}
		}else
		{
			break;
		}
	}
	return freeSpace;
}

double RobotOperator::evaluateAction(double direction, double velocity, bool debug)
{
	sensor_msgs::PointCloud* originalCloud = getPointCloud(direction, velocity);
	sensor_msgs::PointCloud transformedCloud;
	try
	{
		mTfListener.transformPointCloud(mOdometryFrame,*originalCloud,transformedCloud);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return 1;
	}
	
	double valueSafety = 0.0;      // How safe is it to move in that direction?
	double valueEscape = 0.0;      // How much does the safety improve?
	double valueConformance = 0.0; // How conform is it with the desired direction?
	double valueContinue = 0.0;    // How conform is it with the previous command?
	
	double decay = 1.0;
	double safe_max = 0.0;
	double cost_max = 0.0;
	double cost_start = 1.0;
	
	// Calculate safety and escape value
	int length = transformedCloud.points.size();
	for(int i = 0; i < length; i++)
	{
		unsigned int mx, my;
		double cell_cost;
		if(mCostmap->worldToMap(transformedCloud.points[i].x, transformedCloud.points[i].y, mx, my))
		{
			cell_cost = (double)mCostmap->getCost(mx,my) / costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
			if(cell_cost >= 1.0)
			{
				// Trajectory hit an obstacle
				break;
			}
		}
		if(i == 0)
			cost_start = cell_cost;
		double cost = cell_cost * decay;
		double safe = (cost_start - cell_cost) * decay * 2.0;
		
		if(cost > cost_max) cost_max = cost;
		if(safe > safe_max) safe_max = safe;
		
		decay *= mSafetyDecay;
	}
	
	double action_value = 0.0;
	double normFactor = 0.0;
	
	// Add safety value
	valueSafety = 1.0 - cost_max;
	action_value += valueSafety * mSafetyWeight;
	normFactor += mSafetyWeight;
	
	// Add escape value
	valueEscape = safe_max;
	action_value += valueEscape * mEscapeWeight;
	normFactor += mEscapeWeight;

	if(mRecoverySteps == 0)
	{
		// Calculate continuety value
		valueContinue = (mCurrentDirection - direction) * 0.5;
		valueContinue = 1.0 - (valueContinue * valueContinue);
		
		// Calculate conformance value
		double corr = (mDesiredDirection - direction) * PI;
		valueConformance = 0.5 * cos(corr) + 0.5;
		
		// Add both to action value
		action_value += valueConformance * mConformanceWeight;
		action_value += valueContinue * mContinueWeight;
		normFactor += mConformanceWeight + mContinueWeight;

		if(mode_switched)
			mode_switched = false;
	}
	
	action_value /= normFactor;
	
	if(debug)
	{
		geometry_msgs::Vector3 cost_msg;
		cost_msg.x = valueSafety;
		cost_msg.y = valueEscape;
		cost_msg.z = valueConformance;
		mCostPublisher.publish(cost_msg); 
	}
	
	return action_value;
}

double diff(double v1, double v2)
{
	if(v1 > v2)
		return v1 - v2;
	else
		return v2 - v1;
}

double RobotOperator::findBestDirection()
{
	double best_dir = -1.0;
	double best_value = 0.0;
	double step = 0.01;
	double dir = -1.0;
	
	while(dir <= 1.0)
	{
		double value = evaluateAction(dir, mDesiredVelocity);
		if(value > best_value)
		{
			best_dir = dir;
			best_value = value;
		}
		dir += step;
	}
	return best_dir;
}

sensor_msgs::PointCloud* RobotOperator::getPointCloud(double direction, double velocity)
{
	if(direction < -1) direction = -1;
	if(direction > 1) direction = 1;
	int offset = (velocity >= 0) ? LUT_RESOLUTION : 3*LUT_RESOLUTION + 1;
	int table_index = (direction * LUT_RESOLUTION) + offset;
	return mTrajTable[table_index];
}