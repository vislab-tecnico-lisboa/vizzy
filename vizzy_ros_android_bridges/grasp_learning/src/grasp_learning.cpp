#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include <sstream>
#include "GazeClient.h"
#include <time.h> 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <tf/transform_listener.h>

static const std::string OPENCV_WINDOW_L = "Left Image";
static const std::string OPENCV_WINDOW_R = "Right Image";
static const std::string OPENCV_WINDOW_COLOR_CONTROLS = "Color Controls";
bool visible_L=false,visible_R=false, servoing=false;
cv_bridge::CvImagePtr cv_ptr;
geometry_msgs::Pose pose;
int x_L=0,y_L=0,x_R=0,y_R=0,width=640,height=480;
double PI=3.1416;
double sphere_azimut=PI/2,sphere_polar=PI/2,sphere_radius=0.8;
int TOL=2;
double INCREMENT=0.01,fixationX=0,fixationY=0,fixationZ=0;
int iLowH = 116,iHighH = 179,iLowS = 129,iHighS = 214,iLowV = 148,iHighV = 255, iErode=1,iDilate=6;

void callbackVoice(const std_msgs::Int8::ConstPtr& msg);
void callbackSavIm(const std_msgs::Bool::ConstPtr& msg);
void callbackGrasp(const std_msgs::Float32MultiArray::ConstPtr& msg);
void callbackImage_L(const sensor_msgs::ImageConstPtr& msg);
void callbackImage_R(const sensor_msgs::ImageConstPtr& msg);
bool msg2Mat(cv::Mat &img, sensor_msgs::ImageConstPtr msg);
cv::Mat imageProcessing(cv::Mat imgIn,bool &visible, int &p_x, int &p_y);
void adjustHeadPosition(void);
bool adjustHeadPositionPolar(void);
bool adjustHeadPositionAzimut(void);
bool adjustHeadPositionRadius(void);
void moveHead(double x, double y, double z);
bool moveArm();
double absolute(double in);
void transformPoint(const tf::TransformListener& listener);

int main(int argc, char **argv){
    ros::init(argc, argv, "grasp_learning");
    ros::NodeHandle n;
    ros::Subscriber subVoice = n.subscribe("voiceCMD", 1000, callbackVoice);
    ros::Subscriber subGrasp = n.subscribe("graspCMD", 1000, callbackGrasp);
    ros::Subscriber subSavIm = n.subscribe("savImCMD", 1000, callbackSavIm);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub_L = it.subscribe("/vizzy/l_camera/image_color", 1, callbackImage_L);
    image_transport::Subscriber image_sub_R = it.subscribe("/vizzy/r_camera/image_color", 1, callbackImage_R);
    cv::namedWindow(OPENCV_WINDOW_L);
    cv::namedWindow(OPENCV_WINDOW_R);
    cv::namedWindow(OPENCV_WINDOW_COLOR_CONTROLS);
    cv::createTrackbar("LowH"  , OPENCV_WINDOW_COLOR_CONTROLS, &iLowH  , 179);
    cv::createTrackbar("HighH" , OPENCV_WINDOW_COLOR_CONTROLS, &iHighH , 179);
    cv::createTrackbar("LowS"  , OPENCV_WINDOW_COLOR_CONTROLS, &iLowS  , 255);
    cv::createTrackbar("HighS" , OPENCV_WINDOW_COLOR_CONTROLS, &iHighS , 255);
    cv::createTrackbar("LowV"  , OPENCV_WINDOW_COLOR_CONTROLS, &iLowV  , 255);
    cv::createTrackbar("HighV" , OPENCV_WINDOW_COLOR_CONTROLS, &iHighV , 255);
    cv::createTrackbar("Erode" , OPENCV_WINDOW_COLOR_CONTROLS, &iErode , 6  );
    cv::createTrackbar("Dilate", OPENCV_WINDOW_COLOR_CONTROLS, &iDilate, 6  );
    ROS_INFO("Grasp Learning");    
    ros::Rate loop_rate(10);

    tf::TransformListener listener(ros::Duration(1));
    ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener)));

     while (ros::ok()){ 
        if(servoing)
            adjustHeadPosition();
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyWindow(OPENCV_WINDOW_L);
    cv::destroyWindow(OPENCV_WINDOW_R);
    cv::destroyWindow(OPENCV_WINDOW_COLOR_CONTROLS);
    return 0;
}

void callbackVoice(const std_msgs::Int8::ConstPtr& msg){
    ROS_INFO("Voice command received");
    int voiceCMD=(int) msg->data;
}

void callbackSavIm(const std_msgs::Bool::ConstPtr& msg){
    ROS_INFO("Save Image command received:");
    bool savImReceived = (bool) msg->data;  
    servoing=true; 
}

void callbackGrasp(const std_msgs::Float32MultiArray::ConstPtr& msg){
    ROS_INFO("Grasp command received:");
    //int graspBox[4], graspPos[2];
    //graspBox[0]=(int) msg->data[0];
    //graspBox[1]=(int) msg->data[1];
    //graspBox[2]=(int) msg->data[2];
    //graspBox[3]=(int) msg->data[3];
    //graspPos[0]=(int) msg->data[4];
    //graspPos[1]=(int) msg->data[5];

    pose.orientation.x = (float) msg->data[6];
    pose.orientation.y = (float) msg->data[7];
    pose.orientation.z = (float) msg->data[8];
    pose.orientation.w = (float) msg->data[9];

    moveArm();
}

void callbackImage_L(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat image;
    if(msg2Mat(image,msg)){
       imageProcessing(image,visible_L,x_L,y_L);
       cv::line(image,cv::Point(width/2,0),cv::Point(width/2,height),cv::Scalar(255,0,0));
       cv::line(image,cv::Point(0,height/2),cv::Point(width,height/2),cv::Scalar(255,0,0));
       if(visible_L)
            cv::circle(image,cv::Point(x_L,y_L),10,cv::Scalar(0,0,255),-1, 8,0);
       cv::imshow(OPENCV_WINDOW_L, image);
       cv::waitKey(3); 
    }
}

void callbackImage_R(const sensor_msgs::ImageConstPtr& msg){
    cv::Mat image;
    if(msg2Mat(image,msg)){
       cv::Mat imageS = imageProcessing(image,visible_R,x_R,y_R);
       cv::imshow(OPENCV_WINDOW_COLOR_CONTROLS, imageS);
       cv::line(image,cv::Point(width/2,0),cv::Point(width/2,height),cv::Scalar(255,0,0));
       cv::line(image,cv::Point(0,height/2),cv::Point(width,height/2),cv::Scalar(255,0,0));
       if(visible_R)
           cv::circle(image,cv::Point(x_R,y_R),10,cv::Scalar(0,0,255),-1, 8,0);
       cv::imshow(OPENCV_WINDOW_R, image);
       cv::waitKey(3); 
    }
}

bool msg2Mat(cv::Mat &img, sensor_msgs::ImageConstPtr msg){
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }

    img=cv_ptr->image;
    return true;
}

cv::Mat imageProcessing(cv::Mat imgIn,bool &visible, int &p_x, int &p_y){
    cv::Mat img;
    cv::cvtColor(imgIn, img, cv::COLOR_BGR2HSV);
    cv::inRange(img, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), img); 
    if(iErode>0)  
       cv::erode (img, img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iErode, iErode)));
    if(iDilate>0)
       cv::dilate(img, img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(iDilate, iDilate))); 

    cv::Moments oMoments = cv::moments(img);

    visible=oMoments.m00 > 10000;

    if (visible){  
       p_x = oMoments.m10 / oMoments.m00;
       p_y = oMoments.m01 / oMoments.m00; 
    }
   
    return img;
}

void adjustHeadPosition(void){
    bool centered_polar=adjustHeadPositionPolar();
    bool centered_azimut=adjustHeadPositionAzimut();
    bool centered_radius=false;
    if(centered_azimut)
        centered_radius=adjustHeadPositionRadius();
    fixationX= sphere_radius*sin(sphere_polar)*cos(sphere_azimut);
    fixationY=-sphere_radius*cos(sphere_polar);
    fixationZ= sphere_radius*sin(sphere_polar)*sin(sphere_azimut);
    moveHead(fixationX,fixationY,fixationZ);
    servoing=!(centered_polar && centered_azimut && centered_radius);
}

bool adjustHeadPositionPolar(void){
    bool centered=false;
    if(visible_L && visible_R){
       double y=(y_L+y_R)/2;
       if(absolute(y-height/2)>TOL/2){
          if(y<height/2)
             sphere_polar-=INCREMENT;
          else
             sphere_polar+=INCREMENT;
       }
       else
          centered=true;
    }
    else
        ROS_INFO("The object is not visible");
    return centered;
}

bool adjustHeadPositionAzimut(void){
    bool centered=false;
    if(visible_L && visible_R){
       int extDistX_L=x_L;
       int extDistX_R=width-x_R;
       if(absolute(extDistX_L-extDistX_R)>TOL/2){
           if(extDistX_L>extDistX_R)
               sphere_azimut-=INCREMENT;
           else
               sphere_azimut+=INCREMENT;
       }
       else
           centered=true;
    }
    else
        ROS_INFO("The object is not visible");
    return centered;
}

bool adjustHeadPositionRadius(void){
    bool centered=false;
    if(visible_L && visible_R){
        if(absolute(x_L-width/2)>TOL/2){
            if(x_L<width/2)
                sphere_radius+=INCREMENT*2;
            else 
                sphere_radius-=INCREMENT*2;
        }
        else
            centered=true;
    }
    else
        ROS_INFO("The object is not visible");
    return centered;
}

void moveHead(double x, double y, double z){
    //ROS_INFO("Move head to x:%f, y:%f, z:%f",x,y,z);
    actionlib::SimpleActionClient<vizzy_msgs::GazeAction> ac("gaze", true); 
    ac.waitForServer();

    vizzy_msgs::GazeGoal goal;
    goal.type=vizzy_msgs::GazeGoal::CARTESIAN;
    goal.fixation_point.point.x = x;
    goal.fixation_point.point.y = y;
    goal.fixation_point.point.z = z;
    goal.fixation_point_error_tolerance = 0.01;
    goal.fixation_point.header.frame_id="ego_frame";
    goal.fixation_point.header.stamp=ros::Time::now();

    ac.sendGoal(goal);
    //ROS_INFO("Move head done");
}

bool moveArm(){
    ROS_INFO("Move arm");
    bool success=true;

    moveit::planning_interface::MoveGroupInterface group("right_arm");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(5);
    group.setStartState(*group.getCurrentState());

    //ROS_INFO("Reference frame: -%s-", group.getPlanningFrame().c_str());
    //ROS_INFO("End Effector Link: -%s-", group.getEndEffectorLink().c_str());
    //ROS_INFO("Position(HEAD) x:%f, y:%f, z:%f",fixationX,fixationY,fixationZ); 
    //ROS_INFO("Pos [x:%f, y:%f, z:%f]",pose.position.x,pose.position.y,pose.position.z); 
    //ROS_INFO("Quat: [w=%f,x=%f,y=%f,z=%f]",pose.position.w,pose.position.x,pose.position.y,pose.orientation.z); 

    success = group.setPoseTarget(pose,"r_palm_link");
    ROS_INFO("Set Position Target: %s", success? "SUCCESS" : "FAILED");

    int maxIterations=5,iteration=0;
    double goalPositionTolerance=0.01;
    double goalOrientationTolerance=PI/180;
    do{
       printf("\n");
       ROS_INFO("Iteration:%d",iteration); 
       group.setGoalPositionTolerance(goalPositionTolerance);
       group.setGoalOrientationTolerance(goalOrientationTolerance);

       ros::AsyncSpinner spinner(1);
       spinner.start();
       moveit::planning_interface::MoveGroupInterface::Plan my_plan;
       moveit::planning_interface::MoveItErrorCode success_ = group.plan(my_plan);
       ROS_INFO("Set Plan: %s", success_? "SUCCESS" : "FAILED");

       if(success){
           moveit::planning_interface::MoveItErrorCode error = moveit::planning_interface::MoveItErrorCode::FAILURE;
           error=group.move();
           if(error!= moveit::planning_interface::MoveItErrorCode::SUCCESS){
               success=false;
               ROS_INFO("Motion FAILED");
           }
       }
       if(!success){
           goalPositionTolerance+=0.01;
           goalOrientationTolerance+=PI/180;
           
       }
       else
           ROS_INFO("Move Arm: SUCCESS");
       iteration++;
    }while(!success && iteration<=maxIterations);

    ROS_INFO("Move arm done");
    return success;
}

double absolute(double in){
    double out=in;
    if(in<0)
       out=-in;
    return out;
}

void transformPoint(const tf::TransformListener& listener){ 
    geometry_msgs::PointStamped head_point;
    head_point.header.frame_id = "ego_frame";
    head_point.header.stamp = ros::Time();
    head_point.point.x = fixationX;
    head_point.point.y = fixationY;
    head_point.point.z = fixationZ;

    geometry_msgs::PointStamped base_point;
    try{
       listener.transformPoint("base_footprint", head_point, base_point);
    }
    catch(tf::TransformException& ex){
       ROS_ERROR("Exception transform \"ego_frame\" -> \"base_footprint\": %s", ex.what()); 
    } 
    pose.position.x = base_point.point.x;
    pose.position.y = base_point.point.y;
    pose.position.z = base_point.point.z;

}
