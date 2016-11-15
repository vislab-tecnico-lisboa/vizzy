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
#include "trajectory_msgs_JointTrajectory.h"
#include "std_msgs_Bool.h"
#include "std_msgs_Int16.h"
#include <math.h>
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

int jnts = 0;  // joint number
int client_status=-1;

class Thread1 : public Thread {
public:
    //    Thread1():Thread(){}
    Thread1(){}
    Thread1(yarp::os::Subscriber<std_msgs_Bool> *my_topic__){
        setSubscriber(my_topic__);
    }
    virtual bool threadInit()
    {
        //printf("Starting thread1\n");
        Time::delay(0.01);
        return true;
    }
    virtual void setSubscriber(yarp::os::Subscriber<std_msgs_Bool> *my_topic__){
        my_topic = my_topic__;
    }
    //called by start after threadInit, s is true iff the thread started
    //successfully
    virtual void run()
    {
        while (!isStopping()) {
            //printf("Hello, from thread1\n");
            std::cout << "Hello from right arm thread:" << std::endl;
            std_msgs_Bool *data;
            data = my_topic->read();
            if (data != NULL && data->data)
                client_status=1;
        }
    }
    virtual void threadRelease()
    {
        printf("Goodbye from thread1\n");
    }
private:
    yarp::os::Subscriber<std_msgs_Bool> *my_topic;
};

int main(int argc, char *argv[]) 
{
    Network yarp;
    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setMonitor(&rep);
    rf.setDefaultContext("vizzyMotorRosInterface");
    rf.setDefaultConfigFile("left_arm_follow_joint_trajectory_sim.ini");
    rf.configure(argc,argv);
    double max_vel;
    int num_joints;
    //Bottle &bGeneral=rf.findGroup("general");
    //bGeneral.setMonitor(rf.getMonitor());
    string robot=rf.find("robot").asString().c_str();
    string part=rf.find("part").asString().c_str();
    num_joints=rf.find("num_joints").asInt();
    max_vel=rf.find("max_vel").asDouble();
    double time_percentage=rf.find("time_percentage").asDouble();
    double time_percentage_last_point =rf.find("time_percentage_last_point").asDouble();
    std::cout << "Time percentage: " << time_percentage << std::endl;
    double goal_angle_threhsold=rf.find("angle_threshold").asDouble();
    std::cout << "Time percentage last point: " << time_percentage_last_point << std::endl;
    Bottle &grp=rf.findGroup("joints_map");
    string multiple_joints = rf.find("multiple_joints").asString().c_str();
    bool multipleJoints;
    if (multiple_joints=="on")
        multipleJoints = true;
    else
        multipleJoints = false;
    std::cout << "size of strings: " << grp.size() << std::endl;
    int sz=grp.size()-1;
    Vector joint_map;
    joint_map.resize(sz,0);
    std::vector<std::string> joints_name_yarp;
    //joints_name_yarp.resize(sz);
    for (int i=0; i<sz; i++){
	joints_name_yarp.push_back(grp.get(1+i).asString());
	std::cout << joints_name_yarp[i] << std::endl;
    }
        //joint_map[i]=grp.get(1+i).asDouble();
    bool min_delta=false;
    std::map<std::string,int> joint_trajectory_map;
    yarp::os::Subscriber<trajectory_msgs_JointTrajectory> subscriber_trajectory_part;
    yarp::os::Subscriber<std_msgs_Bool> subscriber_stop_part;
    yarp::os::Publisher<std_msgs_Int16> publisher_result_part;
    Property options;
    //options.put("robot", "vizzySim");//Needs to be read from a config file
    options.put("robot", "/" + robot);//Needs to be read from a config file
    options.put("device", "remote_controlboard");
    options.put("local", "/" + robot+"/"+ part + "/_pos_interface");
    //options.put("remote", "/vizzySim/right_shoulder_arm");
    options.put("remote", "/" + robot+"/"+ part );
    //Available parts: head torso right_shoulder_arm right_shoulder_arm
    options.put("part", part);
    IPositionControl *ipos=0;
    IPositionControl2 *ipos2=0;
    IPositionDirect  *iposDir=0;
    IVelocityControl2 *vel=0;
    IEncoders *enc=0;
    IPidControl *pid=0;
    IAmplifierControl *amp=0;
    IControlLimits *lim=0;
    IControlLimits2 *lim2 = 0;
    IControlMode2 *iMode2=0;
    IMotor *imot=0;
    ITorqueControl *itorque=0;
    IOpenLoopControl *iopenloop=0;
    IImpedanceControl *iimp=0;
    IInteractionMode *iInteract=0;
    IMotorEncoders *iMotEnc=0;
    IAxisInfo *iInfo = 0;
    PolyDriver dd(options);
    bool ok;
    ok = dd.view(ipos);
    ok &= dd.view(enc);
    ok &= dd.view(iposDir);
    ok &= dd.view(iMode2);

    Thread1 cancel_topic_thread;
    cancel_topic_thread.setSubscriber(&subscriber_stop_part);
    
    //double head_speeds[8] = {30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0};
    double head_speeds[8] = {12.0,12.0,12.0,12.0,12.0,12.0,12.0,12.0};
    int len=sz>0?sz:1;
    double part_speeds[len];
    for (int my_lo_i=0;my_lo_i<len;my_lo_i++){
        part_speeds[my_lo_i]=max_vel;// = {max_vel};
        std::cout << "velocity [" << my_lo_i << "]:" << part_speeds[my_lo_i] << std::endl;
    }
    ipos->setRefSpeeds(part_speeds);
    //ipos->getAxes(&jnts);
    iposDir->getAxes(&jnts);
    if (part == "left_shoulder_arm" || part == "right_shoulder_arm")
        jnts=len;
    printf("Working with %d axes\n", jnts);
    //for (size_t mot_jo=0;mot_jo<jnts;mot_jo++)
    //	iMode2->setControlMode(mot_jo,VOCAB_CM_POSITION_DIRECT);
    double *tmp = new double[jnts];
    double *tmp_read = new double[jnts];

    //int joint_map[8] = {5,3,2,4,0,1,6,7};
    /* creates a node called /yarp/listener */
    Node node("/yarp/"+ part + "_moveit_bridge");


    bool allConnected = false;
    std::vector<bool > portsConnected;
    portsConnected.resize(3);
    int totalConnections=0;
    portsConnected.assign(3,false);
    totalConnections += 3;

    while (!allConnected){
        int connectedNumber=totalConnections;
        if (!portsConnected[0]){
            portsConnected[0] = subscriber_trajectory_part.topic("/" + part+ "_trajectory_from_moveit");
            if (portsConnected[0])
                connectedNumber--;
        }
        if (!portsConnected[1]){
            portsConnected[1] = subscriber_stop_part.topic("/" + part+ "_trajectory_cancel");
            if (portsConnected[1])
                connectedNumber--;
        }
        if (!portsConnected[2]){
            portsConnected[2] = publisher_result_part.topic("/" + part+ "_trajectory_feedback");
            if (portsConnected[2])
                connectedNumber--;
        }
        if (connectedNumber == 0)
            allConnected = true;
        std::cout << "." << std::endl;
        Time::delay(1);
    }
    ok&=cancel_topic_thread.start();
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

    if (!ok) {
        yError("Problems acquiring mandatory interfaces, quitting\n");
        return 1;
    }
    trajectory_msgs_JointTrajectory *traj_data;
    std_msgs_Int16 feedback_msg_to_ros;
    double trajectory_start;
    double trajectory_elapsed_time;
    double expected_trajectory_time;
    double current_error;
    bool trajectory_map_read = false;
    while (true) {
        if (client_status==-1){
            std::cout << "Waiting for trajectory..." << std::endl;
            traj_data = subscriber_trajectory_part.read();
            //traj_data = subscriber_trajectory_part.read();
            client_status = 0;
        }
        else if (client_status==0){
            trajectory_start = Time::now();
            int points_size = traj_data->points.size();
            std::cout << "Number of points: " << points_size << std::endl;
            expected_trajectory_time=0.0;
	    if (!trajectory_map_read){
		for (int my_j=0;my_j<jnts;my_j++){
		    joint_trajectory_map[traj_data->joint_names.at(my_j)]=my_j;
		}
		trajectory_map_read=true;
	    }
            for (int my_i=0; my_i<points_size; my_i++){
                enc->getEncoders(tmp_read);
                double delta_ang=-1.0;
                if (min_delta)
                    delta_ang = 10000.0;
                else
                    delta_ang = -1.0;
                for (int my_j=0;my_j<jnts;my_j++){
		    std::map<std::string,int>::iterator it=joint_trajectory_map.find(joints_name_yarp[my_j]);
                    //tmp[my_j] = traj_data->points.at(my_i).positions.at(joint_map[my_j])*180.0/3.141592;
		    tmp[my_j] = traj_data->points.at(my_i).positions.at(it->second)*180.0/3.141592;
                    double curr_delta_ang = fabs(tmp[my_j] - tmp_read[my_j]);
                    //std::cout << " delta joint [" << my_j << "] : " << curr_delta_ang << std::endl;
                    if (curr_delta_ang >= delta_ang){
                        if (!min_delta)
                            delta_ang = curr_delta_ang;
                    }
                    else
                        if(min_delta)
                            delta_ang = curr_delta_ang;
                    //std::cout << "Motion to :" << data->data[my_i]*180.0/3.141592 << "sent!" << std::endl;
                }

                if (multipleJoints)//multiple joints case
                    ipos->positionMove(tmp);
                else{//single joint execution
                    for (int my_j=0;my_j<jnts;my_j++){
                        ipos->positionMove(my_j,tmp[my_j]);
                        //iposDir->setPosition(my_j,tmp[my_j]);
                    }
                }
                double current_delay;
                //if (my_i!=points_size-1)
                current_delay = delta_ang/part_speeds[0];
                //else
                //    current_delay = (5/30);
                //std::cout << "current delay: " << current_delay << " and percentage: " << current_delay*(time_percentage) << std::endl;
                //Time::delay(current_delay*0.6);
                //Time::delay(current_delay*(1.0/3.0));
                Time::delay(current_delay*(time_percentage));
                //Time::delay(0.01);
                if (my_i==points_size-1)
                    Time::delay(current_delay*time_percentage_last_point);
                /*if (my_i==points_size-1){
        bool motionDone=false;
        while (motionDone==false){
          //multiple joints check motion done
          //ipos->checkMotionDone(&motionDone);
          bool currentMotionDone;
          bool previousMotionDone=true;
          for (int my_j=0;my_j<jnts;my_j++){
            ipos->checkMotionDone(my_j,&currentMotionDone);
            currentMotionDone &= previousMotionDone;
            previousMotionDone = currentMotionDone;
          }
          motionDone = currentMotionDone;
          if (motionDone)
            std::cout << "Motion done! "<< std::endl;
          else
            std::cout << "Not done yet! " << std::endl;
        }
        }*/
                expected_trajectory_time+=current_delay*0.6;
                enc->getEncoders(tmp_read);
                trajectory_elapsed_time = Time::now();
                current_error=0.0;
                for (int my_j=0;my_j<jnts;my_j++){
                    current_error += fabs(traj_data->points.at(points_size-1).positions.at(joint_map[my_j])*180.0/3.141592-tmp_read[my_j]);
                    //std::cout << "Motion to :" << data->data[my_i]*180.0/3.141592 << "sent!" << std::endl;
                }
                //std::cout << "error at point [" << my_i << " ]:" << current_error << std::endl;
                /*if (expected_trajectory_time/(trajectory_elapsed_time-trajectory_start) < 0.7){
            feedback_msg_to_ros.data = 0;
            publisher_result_part.write(feedback_msg_to_ros);
            client_status=-1;
            break;
        }*/
            }
            //Time::delay(0.09);
            enc->getEncoders(tmp_read);
            for (int my_j=0;my_j<jnts;my_j++){
                current_error += fabs(traj_data->points.at(points_size-1).positions.at(joint_map[my_j])*180.0/3.141592-tmp_read[my_j]);
                //std::cout << "Motion to :" << data->data[my_i]*180.0/3.141592 << "sent!" << std::endl;
            }
            std::cout << "error at last trajectory point:" << current_error << std::endl;
            //if (expected_trajectory_time/(trajectory_elapsed_time-trajectory_start) < 0.7 && current_error<4.0){
            if (current_error<goal_angle_threhsold){
                feedback_msg_to_ros.data = 1;
                publisher_result_part.write(feedback_msg_to_ros);
                client_status = -1;
                std::cout << "succesful: " << std::endl;
            }
            else{
                feedback_msg_to_ros.data = 0;
                publisher_result_part.write(feedback_msg_to_ros);
                std::cout << "failed! " << std::endl;
                client_status = -1;
            }
            //std::cout << "error: " << current_error << std::endl;
        }
        else if (client_status==1){
            client_status = -1;
        }
        std::cout << "client_status: " << client_status << std::endl;
    }
    return 0;
}
