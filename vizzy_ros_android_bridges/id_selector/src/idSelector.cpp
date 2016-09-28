#include "ros/ros.h"
#include <id_selector/BBList.h>
#include <id_selector/BoundingBox.h>
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include <sstream>

bool msg1Received = false,msg2Received = false;
int pixel[2] = {0,0};
int voiceCMD=0;
int selectedID = 1000;
std::vector<id_selector::BoundingBox> personsList;
int listSize;
std::string msgStr;

void callback1(const id_selector::BBList::ConstPtr& msg){
  personsList =  msg->bbVector;
  listSize = personsList.size();
}

void callback2(const std_msgs::Int16MultiArray::ConstPtr& msg){
  pixel[0]=(int) msg->data[0];
  pixel[1]=(int) msg->data[1];
  msg1Received = true;
}

void callback3(const std_msgs::Int8::ConstPtr& msg){
  voiceCMD=(int) msg->data;
  msg2Received = true;
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

int main(int argc, char **argv){
  ros::init(argc, argv, "idSelector");
  ros::NodeHandle n;
  ros::Publisher  pub1 = n.advertise<visualization_msgs::InteractiveMarkerFeedback>("tracker/feedback", 1000);
  ros::Publisher  pub2 = n.advertise<std_msgs::String>("babbler", 1000);
  ros::Subscriber sub1 = n.subscribe("bbs_with_id", 1000, callback1);
  ros::Subscriber sub2 = n.subscribe("pixelXY", 1000, callback2);
  ros::Subscriber sub3 = n.subscribe("voiceCMD", 1000, callback3);
  ros::Rate loop_rate(10);
  visualization_msgs::InteractiveMarkerFeedback selectedIDmsg;
  selectedIDmsg.header.stamp = ros::Time::now();
  selectedIDmsg.header.frame_id = "/world";
  std_msgs::String msgLabel;
  int followingID=-1;
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
    if(msg2Received){
      ROS_INFO("Voice command received: %d",voiceCMD);
      msg2Received = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
