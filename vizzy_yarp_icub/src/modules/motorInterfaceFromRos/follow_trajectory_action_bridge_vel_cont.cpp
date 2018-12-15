#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>

#include <string>
#include "control_msgs_FollowJointTrajectoryGoal.h"
#include "control_msgs_FollowJointTrajectoryFeedback.h"
#include <ecl/geometry.hpp>
#define PI 3.14159265359
#define DEG_TO_RAD (M_PI / 180)
#define RAD_TO_DEG (180 / M_PI)
#define KP 50.0
#define KV 25.0
#define LARGE_ACTUATOR_VELOCITY 0.8378 //maximum velocity of large actuator (joints 1-3) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 1.0572 //maximum velocity of small actuator (joints 4-6) (rad/s)
#define TIME_SCALING_FACTOR 1.5        //keep the trajectory at a followable speed

#include "Bool.h"
#include "std_msgs_Int16.h"
#include <math.h>
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

int jnts = 0; // joint number
int client_status = -1;

class Thread1 : public Thread
{
  public:
    //    Thread1():Thread(){}
    Thread1() {}
    Thread1(yarp::os::Subscriber<Bool> *my_topic__)
    {
        setSubscriber(my_topic__);
    }
    virtual bool threadInit()
    {
        //printf("Starting thread1\n");
        Time::delay(0.01);
        return true;
    }
    virtual void setSubscriber(yarp::os::Subscriber<Bool> *my_topic__)
    {
        my_topic = my_topic__;
    }
    //called by start after threadInit, s is true iff the thread started
    //successfully
    virtual void run()
    {
        while (!isStopping())
        {
            //printf("Hello, from thread1\n");
            std::cout << "Hello from right arm thread:" << std::endl;
            Bool *data;
            data = my_topic->read();
            if (data != NULL && data->data)
                client_status = 1;
        }
    }
    virtual void threadRelease()
    {
        printf("Goodbye from thread1\n");
    }

  private:
    yarp::os::Subscriber<Bool> *my_topic;
};

int main(int argc, char *argv[])
{
    Network yarp;
    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setMonitor(&rep);
    rf.setDefaultContext("vizzyMotorRosInterface");
    rf.setDefaultConfigFile("left_arm_follow_joint_trajectory_vel_cont_sim.ini");
    rf.configure(argc, argv);
    double max_acc;
    int num_joints;
    //Bottle &bGeneral=rf.findGroup("general");
    //bGeneral.setMonitor(rf.getMonitor());
    string robot = rf.find("robot").asString().c_str();
    string part = rf.find("part").asString().c_str();
    num_joints = rf.find("num_joints").asInt();
    max_acc = rf.find("max_acc").asDouble();
    double totalError = rf.find("error_threshold_rad").asDouble();
    Bottle &grp = rf.findGroup("joints_map");
    std::cout << "size of strings: " << grp.size() << std::endl;
    int sz = grp.size() - 1;
    Vector joint_map;
    joint_map.resize(sz, 0);
    std::vector<std::string> joints_name_yarp;
    //joints_name_yarp.resize(sz);
    for (int i = 0; i < sz; i++)
    {
        joints_name_yarp.push_back(grp.get(1 + i).asString());
        std::cout << joints_name_yarp[i] << std::endl;
    }
    //joint_map[i]=grp.get(1+i).asDouble();
    bool min_delta = false;
    std::map<std::string, int> joint_trajectory_map;
    yarp::os::Subscriber<control_msgs_FollowJointTrajectoryGoal> subscriber_trajectory_part;
    yarp::os::Subscriber<Bool> subscriber_stop_part;
    yarp::os::Publisher<control_msgs_FollowJointTrajectoryFeedback> publisher_feedback_part;
    yarp::os::Publisher<std_msgs_Int16> publisher_result_bridge_part;
    Property options;
    //options.put("robot", "vizzySim");//Needs to be read from a config file
    options.put("robot", "/" + robot); //Needs to be read from a config file
    options.put("device", "remote_controlboard");
    options.put("local", "/" + robot + "/" + part + "/_vel_interface");
    //options.put("remote", "/vizzySim/right_shoulder_arm");
    options.put("remote", "/" + robot + "/" + part);
    //Available parts: head torso right_shoulder_arm right_shoulder_arm
    options.put("part", part);
    IPositionControl *ipos = 0;
    IPositionControl2 *ipos2 = 0;
    IPositionDirect *iposDir = 0;
    IVelocityControl2 *vel = 0;
    IEncoders *enc = 0;
    IPidControl *pid = 0;
    IAmplifierControl *amp = 0;
    IControlLimits *lim = 0;
    IControlLimits2 *lim2 = 0;
    IControlMode2 *iMode2 = 0;
    IMotor *imot = 0;
    ITorqueControl *itorque = 0;
    //IOpenLoopControl *iopenloop=0;
    IImpedanceControl *iimp = 0;
    IInteractionMode *iInteract = 0;
    IMotorEncoders *iMotEnc = 0;
    IAxisInfo *iInfo = 0;
    PolyDriver dd(options);

    bool ok;
    //ok = dd.view(ipos);
    ok &= dd.view(enc);
    //ok &= dd.view(iposDir);
    ok &= dd.view(iMode2);
    ok &= dd.view(vel);
    Thread1 cancel_topic_thread;
    cancel_topic_thread.setSubscriber(&subscriber_stop_part);

    //double arm_acc[8] = {30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0};
    double arm_acc[8] = {12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0};
    int len = sz > 0 ? sz : 1;
    double part_accels[len];
    for (int my_lo_i = 0; my_lo_i < len; my_lo_i++)
    {
        part_accels[my_lo_i] = max_acc; // = {max_vel};
        std::cout << "acceleration [" << my_lo_i << "]:" << part_accels[my_lo_i] << std::endl;
    }
    vel->setRefAccelerations(part_accels);
    //ipos->getAxes(&jnts);
    //iposDir->getAxes(&jnts);
    vel->getAxes(&jnts);
    if (part == "left_shoulder_arm" || part == "right_shoulder_arm")
        jnts = len;
    printf("Working with %d axes\n", jnts);
    //for (size_t mot_jo=0;mot_jo<jnts;mot_jo++)
    //	iMode2->setControlMode(mot_jo,VOCAB_CM_POSITION_DIRECT);
    double *tmp = new double[jnts];
    double *tmp_read = new double[jnts];
    double *current_joint_pos = new double[jnts];
    //int joint_map[8] = {5,3,2,4,0,1,6,7};
    /* creates a node called /yarp/listener */
    Node node("/yarp/" + part + "_moveit_bridge");

    double max_curvature_ = 20.0;

    bool allConnected = false;
    std::vector<bool> portsConnected;
    portsConnected.resize(4);
    int totalConnections = 0;
    portsConnected.assign(4, false);
    totalConnections += 4;

    while (!allConnected)
    {
        int connectedNumber = totalConnections;
        if (!portsConnected[0])
        {
            portsConnected[0] = subscriber_trajectory_part.topic("/" + part + "_trajectory_from_moveit");
            if (portsConnected[0])
                connectedNumber--;
        }
        if (!portsConnected[1])
        {
            portsConnected[1] = subscriber_stop_part.topic("/" + part + "_trajectory_cancel");
            if (portsConnected[1])
                connectedNumber--;
        }
        if (!portsConnected[2])
        {
            portsConnected[2] = publisher_feedback_part.topic("/" + part + "_trajectory_feedback");
            if (portsConnected[2])
                connectedNumber--;
        }
        if (!portsConnected[3])
        {
            portsConnected[3] = publisher_result_bridge_part.topic("/" + part + "_trajectory_result");
            if (portsConnected[3])
                connectedNumber--;
        }
        if (connectedNumber == 0)
            allConnected = true;
        std::cout << "." << std::endl;
        Time::delay(1);
    }
    ok &= cancel_topic_thread.start();
    /*ok &= dd.view(ipos2);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(lim);
    ok &= dd.view(lim2);
//    ok &= dd.view(icm);
    ok &= dd.view(itorque);
    ok &= dd.view(iopenloop);
    ok &= dd.view(iimp);
    ok &= dd.view(iposDir);
    ok &= dd.view(iMode2);
    ok &= dd.view(iInteract);*/

    if (!ok)
    {
        yError("Problems acquiring mandatory interfaces, quitting\n");
        return 1;
    }
    control_msgs_FollowJointTrajectoryGoal *traj_data;
    control_msgs_FollowJointTrajectoryFeedback feedback_msg_to_ros;
    std_msgs_Int16 result_to_ros;
    double trajectory_start;
    double trajectory_elapsed_time;
    double expected_trajectory_time;
    double current_error;
    bool trajectory_map_read = false;
    while (true)
    {
        if (client_status == -1)
        {
            std::cout << "Waiting for trajectory..." << std::endl;
            traj_data = subscriber_trajectory_part.read();
            //traj_data = subscriber_trajectory_part.read();
            client_status = 0;
        }
        else if (client_status == 0)
        {
            trajectory_start = Time::now();
            //traj_data->trajectory.points
            int points_size = traj_data->trajectory.points.size();
            std::cout << "Number of points: " << points_size << std::endl;
            expected_trajectory_time = 0.0;
            if (!trajectory_map_read)
            {
                for (int my_j = 0; my_j < jnts; my_j++)
                {
                    joint_trajectory_map[traj_data->trajectory.joint_names.at(my_j)] = my_j;
                }
                //trajectory_map_read=true;
            }

            float trajectoryPoints[jnts][points_size];
            for (unsigned int i = 0; i < points_size; i++)
            {
                for (int my_j = 0; my_j < jnts; my_j++)
                {
                    std::map<std::string, int>::iterator it = joint_trajectory_map.find(joints_name_yarp[my_j]);
                    //tmp[my_j] = traj_data->points.at(my_i).positions.at(joint_map[my_j])*180.0/3.141592;
                    trajectoryPoints[i][my_j] = traj_data->trajectory.points.at(i).positions.at(it->second);
                    //tmp[my_j] = traj_data->points.at(my_i).positions.at(it->second)*180.0/3.141592;
                }
            }
            //initialize arrays needed to fit a smooth trajectory to the given points
            ecl::Array<double> timePoints(points_size);
            timePoints[0] = 0.0;
            vector<ecl::Array<double> > jointPoints;
            jointPoints.resize(jnts);
            float prevPoint[jnts];
            for (unsigned int i = 0; i < jnts; i++)
            {
                jointPoints[i].resize(points_size);
                jointPoints[i][0] = trajectoryPoints[i][0];
                prevPoint[i] = trajectoryPoints[i][0];
            }

            //determine time component of trajectories for each joint
            for (unsigned int i = 1; i < points_size; i++)
            {
                float maxTime = 0.0;
                for (unsigned int j = 0; j < jnts; j++)
                {
                    //calculate approximate time required to move to the next position
                    float time = fabs(trajectoryPoints[j][i] - prevPoint[j]);
                    if (j <= 2)
                        time /= LARGE_ACTUATOR_VELOCITY;
                    else
                        time /= SMALL_ACTUATOR_VELOCITY;

                    if (time > maxTime)
                        maxTime = time;

                    jointPoints[j][i] = trajectoryPoints[j][i];
                    prevPoint[j] = trajectoryPoints[j][i];
                }

                timePoints[i] = timePoints[i - 1] + maxTime * TIME_SCALING_FACTOR;
            }

            vector<ecl::SmoothLinearSpline> splines;
            splines.resize(6);
            for (unsigned int i = 0; i < jnts; i++)
            {
                ecl::SmoothLinearSpline tempSpline(timePoints, jointPoints[i], max_curvature_);
                splines.at(i) = tempSpline;
            }

            //control loop
            bool trajectoryComplete = false;
            double startTime = Time::now();
            double t = 0;
            float error[jnts];
            float totalError;
            float prevError[jnts] = {0};
            float currentPoint;
            //trajPoint has to be replaced with the velocity commands vector to be sent
            //AngularInfo trajPoint;
            //trajPoint.InitStruct();
            //Rate has to be set properly for Vizzy
            double rate = 600;
            //ros::Rate rate(rate_hz_);

            while (!trajectoryComplete)
            {
                /*if (arm_comm_.isStopped())
    {
      control_msgs::FollowJointTrajectoryResult result;
            ROS_INFO("Could not complete joint angle action because the arm is 'stopped'.");
            result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
            action_server_.setSucceeded(result);
            return;
    }*/

                /////This needs to be checked at some point!!!
                //check for preempt requests from clients
                /*if (action_server_.isPreemptRequested())
    {
      //stop gripper control
      trajPoint.Actuator1 = 0.0;
      trajPoint.Actuator2 = 0.0;
      trajPoint.Actuator3 = 0.0;
      trajPoint.Actuator4 = 0.0;
      trajPoint.Actuator5 = 0.0;
      trajPoint.Actuator6 = 0.0;
      arm_comm_.setJointVelocities(trajPoint);
      //executeAngularTrajectoryPoint(trajPoint, true);

      //preempt action server
      action_server_.setPreempted();
      ROS_INFO("Joint trajectory server preempted by client");

      return;
    }*/

                //get time for trajectory
                t = Time::now() - startTime;
                if (t > timePoints.at(timePoints.size() - 1))
                {
                    //use final trajectory point as the goal to calculate error until the error
                    //is small enough to be considered successful
                    //arm_comm_.getJointAnglesNotNormalized(current_joint_angles);
                    enc->getEncoders(current_joint_pos);
                    for (int my_j = 0; my_j < jnts; my_j++)
                    {
                        current_joint_pos[my_j] *= DEG_TO_RAD;
                    }
                    /*current_joint_pos[0] = current_joint_angles.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = current_joint_angles.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = current_joint_angles.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = current_joint_angles.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = current_joint_angles.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = current_joint_angles.Actuator6 * DEG_TO_RAD;*/

                    totalError = 0;
                    for (unsigned int i = 0; i < jnts; i++)
                    {
                        error[i] = (splines.at(i))(timePoints.at(timePoints.size() - 1)) - current_joint_pos[i];
                        totalError += fabs(error[i]);
                    }

                    if (totalError < .03)
                    {

                        for (int my_j = 0; my_j < jnts; my_j++)
                        {
                            tmp[my_j] = 0.0;
                        }
                        /*trajPoint.Actuator2 = 0.0;
        trajPoint.Actuator3 = 0.0;
        trajPoint.Actuator4 = 0.0;
        trajPoint.Actuator5 = 0.0;
        trajPoint.Actuator6 = 0.0;
	arm_comm_.setJointVelocities(trajPoint);*/
                        trajectoryComplete = true;
                        yWarning("Trajectory complete!");
                        //Exit while
                        break;
                    }
                }
                else
                {
                    //calculate error
                    /*arm_comm_.getJointAnglesNotNormalized(current_joint_angles);
      current_joint_pos[0] = current_joint_angles.Actuator1 * DEG_TO_RAD;
      current_joint_pos[1] = current_joint_angles.Actuator2 * DEG_TO_RAD;
      current_joint_pos[2] = current_joint_angles.Actuator3 * DEG_TO_RAD;
      current_joint_pos[3] = current_joint_angles.Actuator4 * DEG_TO_RAD;
      current_joint_pos[4] = current_joint_angles.Actuator5 * DEG_TO_RAD;
      current_joint_pos[5] = current_joint_angles.Actuator6 * DEG_TO_RAD;*/
                    enc->getEncoders(current_joint_pos);
                    for (int my_j = 0; my_j < jnts; my_j++)
                    {
                        current_joint_pos[my_j] *= DEG_TO_RAD;
                    }
                    for (unsigned int i = 0; i < jnts; i++)
                    {
                        error[i] = (splines.at(i))(t)-current_joint_pos[i];
                    }
                }

                //calculate control input
                //populate the velocity command
                for (int my_j = 0; my_j < jnts; my_j++)
                {
                    tmp[my_j] = (KP * error[my_j] + KV * (error[my_j] - prevError[my_j]) * RAD_TO_DEG);
                }
                /*trajPoint.Actuator1 = (KP * error[0] + KV * (error[0] - prevError[0]) * RAD_TO_DEG);
    trajPoint.Actuator2 = (KP * error[1] + KV * (error[1] - prevError[1]) * RAD_TO_DEG);
    trajPoint.Actuator3 = (KP * error[2] + KV * (error[2] - prevError[2]) * RAD_TO_DEG);
    trajPoint.Actuator4 = (KP * error[3] + KV * (error[3] - prevError[3]) * RAD_TO_DEG);
    trajPoint.Actuator5 = (KP * error[4] + KV * (error[4] - prevError[4]) * RAD_TO_DEG);
    trajPoint.Actuator6 = (KP * error[5] + KV * (error[5] - prevError[5]) * RAD_TO_DEG);*/

                //for debugging:
                //cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;

                //send the velocity command
                //arm_comm_.setJointVelocities(trajPoint);//AngularInfo &joint_vel
                vel->velocityMove(tmp);
                for (unsigned int i = 0; i < jnts; i++)
                {
                    prevError[i] = error[i];
                }

                yarp::os::Time::delay(1 / rate); //rate.sleep();
            }

            if (trajectoryComplete)
            {
                result_to_ros.data = 1;
                publisher_feedback_part.write(result_to_ros);
                client_status = -1;
                std::cout << "succesful: " << std::endl;
            }
            else
            {
                result_to_ros.data = 0;
                publisher_feedback_part.write(result_to_ros);
                std::cout << "failed! " << std::endl;
                client_status = -1;
            }
            //std::cout << "error: " << current_error << std::endl;
        }
        else if (client_status == 1)
        {
            client_status = -1;
        }
        std::cout << "client_status: " << client_status << std::endl;
    }
    return 0;
}
