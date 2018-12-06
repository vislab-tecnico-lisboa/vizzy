

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include "Bool.h"
#include "geometry_msgs_Pose.h"
#include "Int16.h"
//#include <yarp/os/Subscriber.h>
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

int jnts = 0; // joint number
int client_status = -1;

class StatusThread : public Thread
{
  public:
    //    Thread1():Thread(){}
    StatusThread() {}
    StatusThread(yarp::os::Subscriber<Bool> *my_topic__)
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
            std::cout << "Hello from cartesian cancel bridge thread:" << std::endl;
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
    rf.setDefaultContext("vizzyMotorRosInterface");
    rf.setDefaultConfigFile("cartesian_action_bridge.ini");
    rf.configure(argc, argv);
    PolyDriver client;
    ICartesianControl *arm;
    IPositionControl *ipos;
    IPositionControl *ipos_torso;
    PolyDriver dd;
    PolyDriver dd_torso;
    int n_joints;
    IControlMode2 *iMode2;
    IEncoders *arm_encs;
    IControlMode2 *iMode2_torso;
    VectorOf<int> jntArm;
    yarp::os::Subscriber<geometry_msgs_Pose> pose_reading_port;
    StatusThread *pose_status_thread;
    double home_joint_position[8];
    double current_encoders[8];
    string robot = rf.find("robot").asString().c_str();
    string part = rf.find("part").asString().c_str();
    string remote_port = rf.find("remote").asString().c_str();
    string local_port = rf.find("local").asString().c_str();
    double position_error_threshold = rf.find("position_error_threshold").asDouble();
    Bottle &grp1=rf.findGroup("home_joint_position");
    for (int i=0; i<8; i++){
        home_joint_position[i]=grp1.get(1+i).asDouble();
        std::cout << "Home joint position [" << i << "]: " << home_joint_position[i] << std::endl;
    }
    Property option("(device cartesiancontrollerclient)");
    option.put("remote", ("/" + remote_port).c_str());
    option.put("local", ("/" + local_port).c_str());
    Property options;
    options.put("robot", robot); //Needs to be read from a config file
    options.put("device", "remote_controlboard");
    options.put("remote", "/" + robot + "/" + part);
    options.put("local", "/" + robot + "/" + part + "/_pos_interface");
    options.put("part", part);

    Property options_torso;
    options_torso.put("robot", robot); //Needs to be read from a config file
    options_torso.put("device", "remote_controlboard");
    options_torso.put("remote", "/" + robot + "/" + "torso");
    options_torso.put("local", "/" + robot + "/" + "torso" + "/_pos_interface");
    options_torso.put("part", "torso");

    dd.open(options);
    cout << "Arm driver done!!" << endl;
    dd_torso.open(options_torso);
    cout << "Torso driver done!!" << endl;
    yarp::os::Subscriber<geometry_msgs_Pose> subscriber_pose_part;
    yarp::os::Subscriber<Bool> subscriber_cancel_part;
    yarp::os::Publisher<geometry_msgs_Pose> publisher_feedback_part;
    yarp::os::Publisher<Int16> publisher_result_bridge_part;
    bool ok = true;
    ok &= client.open(option);
    ok &= dd.view(ipos);
    ok &= dd.view(arm_encs);
    ok &= dd.view(iMode2);
    ok &= dd_torso.view(ipos_torso);
    ok &= dd_torso.view(iMode2_torso);
    double part_speeds[8] = {12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0};
    ipos->setRefSpeeds(part_speeds);
    //cout << "Init four done!!" << endl;
    // open the view
    client.view(arm);
    for (int i = 0; i < 8; i++)
        jntArm.push_back(i);
    Vector dof;
    arm->getDOF(dof);
    if (!ok)
        return 0;
    StatusThread cancel_topic_thread;
    cancel_topic_thread.setSubscriber(&subscriber_cancel_part);

    bool allConnected = false;
    std::vector<bool> portsConnected;
    portsConnected.resize(4);
    int totalConnections = 0;
    portsConnected.assign(4, false);
    totalConnections += 4;
    Node node("/yarp/"+ part + "_cartesian_bridge");
    while (!allConnected)
    {
        int connectedNumber = totalConnections;
        if (!portsConnected[0])
        {
            portsConnected[0] = subscriber_pose_part.topic("/" + part + "_cartesian_pose_from_ros");
            if (portsConnected[0])
                connectedNumber--;
        }
        if (!portsConnected[1])
        {
            portsConnected[1] = subscriber_cancel_part.topic("/" + part + "_cartesian_pose_cancel");
            if (portsConnected[1])
                connectedNumber--;
        }
        if (!portsConnected[2])
        {
            portsConnected[2] = publisher_result_bridge_part.topic("/" + part + "_cartesian_pose_result");
            if (portsConnected[2])
                connectedNumber--;
        }
        if (!portsConnected[3])
        {
            portsConnected[3] = publisher_feedback_part.topic("/" + part + "_cartesian_pose_feedback");
            if (portsConnected[3])
                connectedNumber--;
        }
        if (connectedNumber == 0)
            allConnected = true;
        std::cout << "." << std::endl;
        Time::delay(1);
    }
    ok &= cancel_topic_thread.start();
    if (!ok)
        return 0;
    geometry_msgs_Pose *pose_data;
    geometry_msgs_Pose feedback_msg_to_ros;
    Int16 result_msg_to_ros;
    Vector current_position;
    current_position.resize(3);
    Vector current_orientation;
    current_orientation.resize(4);
    geometry_msgs_Pose current_pose;
    double timeout = 3.0;
    while (true)
    {
        if (client_status == -1)
        {
            std::cout << "Waiting for pose..." << std::endl;
            pose_data = subscriber_pose_part.read();
            //traj_data = subscriber_trajectory_part.read();
            if (isnan(pose_data->position.x) && isnan(pose_data->position.y) && isnan(pose_data->position.z))
                client_status = 2;
            else if (isnan(pose_data->orientation.y) && isnan(pose_data->orientation.z) && isnan(pose_data->orientation.w))
                client_status = 3;
            else
                client_status = 0;
        }
        else if (client_status == 0)
        {
            //trajectory_start = Time::now();
            yWarning("Before goToPoseSync");
            Vector position;
            position.resize(3);
            position[0] = pose_data->position.x;
            position[1] = pose_data->position.y;
            position[2] = pose_data->position.z;
            yarp::math::Quaternion orientation(pose_data->orientation.x, pose_data->orientation.y, pose_data->orientation.z, pose_data->orientation.w);
            arm->goToPoseSync(position, orientation.toAxisAngle());
            bool done = false;
            Vector xdhat,odhat, qdhat;
            xdhat.resize(3);
            odhat.resize(4);
            qdhat.resize(9);
            arm->getDesired(xdhat, odhat, qdhat);
            double my_timeout=3.0;
            /*yWarning("Before timeout");
            done = arm->waitMotionDone(0.1, timeout);
            arm->getPose(current_position, current_orientation);
            current_pose.position.x = current_position[0];
            current_pose.position.y = current_position[1];
            current_pose.position.z = current_position[2];
            orientation.fromAxisAngle(current_orientation);
            current_pose.orientation.x = orientation.x();
            current_pose.orientation.y = orientation.y();
            current_pose.orientation.z = orientation.z();
            current_pose.orientation.w = orientation.w();
            publisher_feedback_part.write(current_pose);*/
            while (!done && my_timeout>0)
            {
                //yWarning("Sending the arm to a pose");
                arm->getPose(current_position, current_orientation);
                arm->checkMotionDone(&done);
                current_pose.position.x = current_position[0];
                current_pose.position.y = current_position[1];
                current_pose.position.z = current_position[2];
                orientation.fromAxisAngle(current_orientation);
                current_pose.orientation.x = orientation.x();
                current_pose.orientation.y = orientation.y();
                current_pose.orientation.z = orientation.z();
                current_pose.orientation.w = orientation.w();
                publisher_feedback_part.write(current_pose);
                Time::delay(0.05);
                my_timeout-=0.05;
                //done = true;
            }
            arm->getPose(current_position, current_orientation);
            Vector current_position_error = current_position - xdhat;
            if (norm2(current_position_error) < position_error_threshold)
            {
                result_msg_to_ros.data = 1;
                publisher_result_bridge_part.write(result_msg_to_ros);
                client_status = -1;
                std::cout << "succesful: " << std::endl;
            }
            else
            {
                result_msg_to_ros.data = 0;
                publisher_result_bridge_part.write(result_msg_to_ros);
            }
            //cout << "Initial position: " << home_position[0] << " y: " << home_position[1] << " z:" << home_position[2] << endl;
            //cout << "Initial orientation: or1: " << home_orientation[0] << " or2: "<< home_orientation[1] << " or3: " <<home_orientation[2] << " angle:" << home_orientation[3]<< endl;
        }
        else if (client_status == 1)
        {
            arm->stopControl();
            client_status = -1;
        }
        else if (client_status == 2)
        {
            arm->stopControl();
            //Do position control to home position
            //--
            //BEGIN Setting the motor control in POSITION mode for each joint
            //--
            VectorOf<int> modes;
            modes.resize(8, VOCAB_CM_POSITION);
            iMode2->setControlModes(jntArm.size(), jntArm.getFirst(), modes.getFirst());
            iMode2_torso->setControlMode(0,VOCAB_CM_POSITION);
            //--
            // END Setting the motor control in POSITION mode for each joint
            //--

            //--
            //BEGIN Setting the motor angular positions for each joint
            //--
            //Initial values left arm joints
            //double joints_arm[8] = {-12, 30, 12, -22, 66, -39, 23, 0.0};
            //ipos->positionMove(joints_arm);
            ipos->positionMove(home_joint_position);
            ipos_torso->positionMove(0,0);
            bool motionDone_arm = false;
            double init_time = Time::now();
            double current_time;
            yWarning("Before position control timeout");
            //while (motionDone_arm==false && current_time-init_time<timeout*2.0){
            while (current_time - init_time < timeout * 2.0)
            {
                ipos->checkMotionDone(&motionDone_arm);
                current_time = Time::now();
            }
            yWarning("After position control timeout");
            //--
            //BEGIN Setting the motor control in POSITION_DIRECT mode for each joint
            //--
            //VectorOf<int> modes;
            modes.resize(8, VOCAB_CM_POSITION_DIRECT);
            iMode2->setControlModes(jntArm.size(), jntArm.getFirst(), modes.getFirst());
            iMode2_torso->setControlMode(0,VOCAB_CM_POSITION_DIRECT);
            //--
            // END Setting the motor control in POSITION_DIRECT mode for each joint
            //--
            arm_encs->getEncoders(current_encoders);
            Vector current_position_error;
            current_position_error = current_encoders - home_joint_position;
            arm->getPose(current_position, current_orientation);
            if (norm2(current_position_error) < position_error_threshold)
            {
                result_msg_to_ros.data = 1;
                publisher_result_bridge_part.write(result_msg_to_ros);
                client_status = -1;
                std::cout << "succesful: " << std::endl;
            }
            else
            {
                result_msg_to_ros.data = 0;
                publisher_result_bridge_part.write(result_msg_to_ros);
            }
            arm->stopControl();
            client_status = -1;
        }
        else if (client_status ==3){
            // Do velocity control
            // End do velocity control
            client_status = -1;
        }
        std::cout << "client_status: " << client_status << std::endl;
    }
    return 0;
}
