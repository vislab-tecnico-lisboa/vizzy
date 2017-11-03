#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>

#include <string>
#include "trajectory_msgs_JointTrajectory.h"
#include "std_msgs_Bool.h"
#include "std_msgs_Int16.h"
#include <math.h>
using namespace yarp::dev;
using namespace yarp::os;

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
	std::cout << "Hello from left arm thread:" << std::endl;
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

    yarp::os::Subscriber<trajectory_msgs_JointTrajectory> subscriber_trajectory_left_arm;
    yarp::os::Subscriber<std_msgs_Bool> subscriber_stop_left_arm;
    yarp::os::Publisher<std_msgs_Int16> publisher_result_left_arm;
    Property options;
    //options.put("robot", "vizzySim");//Needs to be read from a config file
    options.put("robot", "vizzy");//Needs to be read from a config file
    options.put("device", "remote_controlboard");
    options.put("local", "/vizzy/left_arm_pos_interface");
    //options.put("remote", "/vizzySim/left_shoulder_arm");
    options.put("remote", "/vizzy/left_shoulder_arm");
    //Available parts: head torso left_shoulder_arm right_shoulder_arm
    options.put("part", "left_shoulder_arm");
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
    //IOpenLoopControl *iopenloop=0;
    IImpedanceControl *iimp=0;
    IInteractionMode *iInteract=0;
    IMotorEncoders *iMotEnc=0;
    IAxisInfo *iInfo = 0;
    PolyDriver dd(options);
    bool ok;
    ok = dd.view(ipos);
    ok &= dd.view(enc);

    Thread1 cancel_topic_thread;
    cancel_topic_thread.setSubscriber(&subscriber_stop_left_arm);
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
    //double head_speeds[8] = {30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0};
    double head_speeds[8] = {12.0,12.0,12.0,12.0,12.0,12.0,12.0,12.0};
    ipos->setRefSpeeds(head_speeds);
    ipos->getAxes(&jnts);
    printf("Working with %d axes\n", jnts);
    double *tmp = new double[jnts];
    double *tmp_read = new double[jnts];
    //int joint_map[8] = {4,5,2,1,3,0,6,7};
    int joint_map[8] = {5,3,2,4,0,1,6,7};
    /* creates a node called /yarp/listener */
    Node node("/yarp/left_arm_moveit_bridge");


  bool allConnected = false;
  std::vector<bool > portsConnected;
  portsConnected.resize(3);
  int totalConnections=0;
  portsConnected.assign(3,false);
  totalConnections += 3;

  while (!allConnected){
    int connectedNumber=totalConnections;
    if (!portsConnected[0]){
      portsConnected[0] = subscriber_trajectory_left_arm.topic("/left_arm_trajectory_from_moveit");
      if (portsConnected[0])
        connectedNumber--;
    }
    if (!portsConnected[1]){
      portsConnected[1] = subscriber_stop_left_arm.topic("/left_arm_trajectory_cancel");
      if (portsConnected[1])
        connectedNumber--;
    }
    if (!portsConnected[2]){
      portsConnected[2] = publisher_result_left_arm.topic("/left_arm_trajectory_feedback");
      if (portsConnected[2])
        connectedNumber--;
    }
  if (connectedNumber == 0)
      allConnected = true;
    std::cout << "." << std::endl;
    Time::delay(1);
  }
  trajectory_msgs_JointTrajectory *traj_data;
  std_msgs_Int16 feedback_msg_to_ros;
  double trajectory_start;
  double trajectory_elapsed_time;
  double expected_trajectory_time;
  double current_error;
  while (true) {
	if (client_status==-1){
	    std::cout << "Waiting for trajectory..." << std::endl;
	    traj_data = subscriber_trajectory_left_arm.read();
	    //traj_data = subscriber_trajectory_left_arm.read();
	    client_status = 0;
	}
	else if (client_status==0){
	    trajectory_start = Time::now();
	    int points_size = traj_data->points.size();
	    std::cout << "Number of points: " << points_size << std::endl;
	    expected_trajectory_time=0.0;
	    for (int my_i=0; my_i<points_size; my_i++){
		enc->getEncoders(tmp_read);
		double delta_ang=-1.0;
		for (int my_j=0;my_j<jnts;my_j++){
		    tmp[my_j] = traj_data->points.at(my_i).positions.at(joint_map[my_j])*180.0/3.141592;
		    double curr_delta_ang = fabs(tmp[my_j] - tmp_read[my_j]);
		    //std::cout << " delta joint [" << my_j << "] : " << curr_delta_ang << std::endl; 
		    if (curr_delta_ang > delta_ang)
			delta_ang = curr_delta_ang;
		    //std::cout << "Motion to :" << data->data[my_i]*180.0/3.141592 << "sent!" << std::endl;
		}
		//multiple joints case
		ipos->positionMove(tmp);
		//single joint execution
		/*for (int my_j=0;my_j<jnts;my_j++){
		  ipos->positionMove(my_j,tmp[my_j]);
		}*/
		double current_delay;
		//if (my_i!=points_size-1)
		current_delay = delta_ang/head_speeds[0];
		//else
		//    current_delay = (5/30);
		//std::cout << "current delay: " << current_delay << std::endl;
		//Time::delay(current_delay*0.6);
		Time::delay(current_delay*1.0);
		//Time::delay(0.01);
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
		    publisher_result_left_arm.write(feedback_msg_to_ros);
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
	    if (current_error<7.0){
		feedback_msg_to_ros.data = 1;
		publisher_result_left_arm.write(feedback_msg_to_ros);
		client_status = -1;
		std::cout << "succesful: " << std::endl;
	    }
	    else{
		feedback_msg_to_ros.data = 0;
		publisher_result_left_arm.write(feedback_msg_to_ros);
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
