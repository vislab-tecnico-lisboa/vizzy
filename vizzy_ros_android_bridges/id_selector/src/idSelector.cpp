#include "ros/ros.h"
#include <id_selector/BBList.h>
#include <id_selector/BoundingBox.h>
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

bool msg1Received = false,msg2Received = false;
int pixel[2] = {0,0};
int voiceCMD=0;
int selectedID = 1000;
std::vector<id_selector::BoundingBox> personsList;
int listSize;
std::string msgStr;
double UP[4]={0.0,0.0,0.0,1.0};
double DOWN[4]={0.0,0.0,1.0,0.0};
double LEFT[4]={0.0,0.0,0.7071067811,0.7071067811};
double RIGHT[4]={0.0,0.0,-0.7071067811,0.7071067811};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void moveBaseToCoordinates(double px, double py,double orientation[]);
void callbackBoundingBox(const id_selector::BBList::ConstPtr& msg);
void callbackPixelCoordinates(const std_msgs::Int16MultiArray::ConstPtr& msg);
void callbackVoice(const std_msgs::Int8::ConstPtr& msg);
void selectID();

int main(int argc, char **argv){
  ros::init(argc, argv, "idSelector");
  ros::NodeHandle n;
  ros::Publisher  pub1 = n.advertise<visualization_msgs::InteractiveMarkerFeedback>("tracker/feedback", 1000);
  ros::Publisher  pub2 = n.advertise<std_msgs::String>("babbler", 1000);
  ros::Subscriber sub1 = n.subscribe("bbs_with_id", 1000, callbackBoundingBox);
  ros::Subscriber sub2 = n.subscribe("pixelXY", 1000, callbackPixelCoordinates);
  ros::Subscriber sub3 = n.subscribe("voiceCMD", 1000, callbackVoice);
  ros::Rate loop_rate(10);
  visualization_msgs::InteractiveMarkerFeedback selectedIDmsg;
  selectedIDmsg.header.stamp = ros::Time::now();
  selectedIDmsg.header.frame_id = "/world";
  std_msgs::String msgLabel;
  int followingID=-1;
  ROS_INFO("ID Selector is running");
  //moveBaseToCoordinates(2.5,-3.0,0.0,0.0,0.7071067811,0.7071067811);
  while (ros::ok()){ 
    if(msg1Received){ 
      if(followingID>-1){  
        std::stringstream int2str;
        int2str << followingID;
        msgStr = "person " + int2str.str();
        ROS_INFO("String sent with ID %d",followingID);
        selectedIDmsg.marker_name=msgStr;
        pub1.publish(selectedIDmsg);
        msgLabel.data=msgStr;
        pub2.publish(msgLabel);
        followingID=-1;
      }
      else{    
        selectID();
        if(selectedID<1000){
          selectedIDmsg.marker_name=msgStr;
          pub1.publish(selectedIDmsg);
          msgLabel.data=msgStr;
          pub2.publish(msgLabel);
          followingID=selectedID;
        }
        else{  
          msgLabel.data=msgStr;
          pub2.publish(msgLabel);
          ROS_INFO("No ID selected");
        }
      }
      msg1Received = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void callbackBoundingBox(const id_selector::BBList::ConstPtr& msg){
  personsList =  msg->bbVector;
  listSize = personsList.size();
}

void callbackPixelCoordinates(const std_msgs::Int16MultiArray::ConstPtr& msg){
  pixel[0]=(int) msg->data[0];
  pixel[1]=(int) msg->data[1];
  msg1Received = true;
}

void callbackVoice(const std_msgs::Int8::ConstPtr& msg){
  voiceCMD=(int) msg->data;
  ROS_INFO("Voice command received: %d",voiceCMD);
  msg2Received = true;
  if(voiceCMD==4){
     ROS_INFO("I will go to the office");     
     moveBaseToCoordinates(-1.0,-15.5,DOWN);
  }
  if(voiceCMD==5){
     ROS_INFO("I will go to the laboratory");  
     moveBaseToCoordinates(0.5,-2.5,LEFT);
  }
  if(voiceCMD==6){
     ROS_INFO("I will go to the meeting room");  
     moveBaseToCoordinates(15.0,-4.5,LEFT);
  }
  if(voiceCMD==7){
     ROS_INFO("I will go to the elevator");  
     moveBaseToCoordinates(0.0,-7.0,UP);
  }
  if(voiceCMD==8){
     ROS_INFO("I will go to the bathroom");  
     moveBaseToCoordinates(8.5,-18.0,LEFT);
  }
}

void selectID(){
  selectedID = 1000;
  int boxXMin,boxXMax,boxYMin,boxYMax;
  for(int k=0; k<listSize;k++){
    boxXMin=personsList[k].x;
    boxXMax=personsList[k].x+personsList[k].width;
    boxYMin=personsList[k].y;
    boxYMax=personsList[k].y+personsList[k].height;
    if(pixel[0]>boxXMin && pixel[0]<boxXMax && pixel[1]>boxYMin && pixel[1]<boxYMax)
      selectedID=personsList[k].id;  
  }
  std::stringstream int2str;
  int2str << selectedID;
  msgStr = "person " + int2str.str();
  ROS_INFO("String sent with ID %d",selectedID);
}

void moveBaseToCoordinates(double px, double py,double orientation[]){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = px;
  goal.target_pose.pose.position.y = py;
  goal.target_pose.pose.orientation.x = orientation[0];
  goal.target_pose.pose.orientation.y = orientation[1];
  goal.target_pose.pose.orientation.z = orientation[2];
  goal.target_pose.pose.orientation.w = orientation[3];

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved");
  else
    ROS_INFO("The base failed to move for some reason");

}
