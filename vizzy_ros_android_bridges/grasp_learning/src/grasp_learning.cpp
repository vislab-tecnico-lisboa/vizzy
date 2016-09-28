#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
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
bool centered_complete=false, servoing=false,visible_L=false,visible_R=false;
cv_bridge::CvImagePtr cv_ptr;
geometry_msgs::Pose pose;
int x_L=0,y_L=0,x_R=0,y_R=0,width=320,height=240;
double PI=3.1416;
double sphere_azimut=PI/2,sphere_polar=PI/2,sphere_radius=0.8;
double angleRange=15,positionRange=5; //angle(degrees), position(cm)
int TOL=2;
double INCREMENT=0.01,fixationX=0,fixationY=0,fixationZ=0;
int iLowH = 116,iHighH = 179,iLowS = 129,iHighS = 214,iLowV = 148,iHighV = 255, iErode=1,iDilate=6;

double randomDouble(double min, double max){
    int minI=(int)(min*100);
    int maxI=(int)(max*100);
    return (double)((rand()%(maxI-minI))+minI)/100.0;
}

void productQuaternion(float q[], float q1[], float q2[]){
   float a1=q1[0],b1=q1[1],c1=q1[2],d1=q1[3];
   float a2=q2[0],b2=q2[1],c2=q2[2],d2=q2[3];
   q[0]=a1*a2-b1*b2-c1*c2-d1*d2;
   q[1]=a1*b2+b1*a2+c1*d2-d1*c2;
   q[2]=a1*c2-b1*d2+c1*a2+d1*b2;
   q[3]=a1*d2+b1*c2-c1*b2+d1*a2;
}

void randomQuaternion(float q[], int axis, double angleRange){
   double angle = randomDouble(-angleRange,angleRange)*PI/180.0;
   q[0]    = (float)cos(angle);
   q[1]    = 0;
   q[2]    = 0;
   q[3]    = 0;
   q[axis] = (float)sin(angle);
   //ROS_INFO("Angle: %f",angle); 
   //ROS_INFO("Axis: %d, Rotation:[w=%f,x=%f,y=%f,z=%f]",axis,q[0],q[1],q[2],q[3]);  
}

void modifyOrientation(float q[]){
    float qR[4]={q[0],q[1],q[2],q[3]};

    float qRx[4],qRy[4],qRz[4];
    randomQuaternion(qRx,1,angleRange); 
    randomQuaternion(qRy,2,angleRange);  
    randomQuaternion(qRz,3,angleRange);  
    productQuaternion(qR,qRz,qR);
    productQuaternion(qR,qRx,qR);
    productQuaternion(qR,qRy,qR);

    pose.orientation.w = qR[0];  
    pose.orientation.x = qR[1];
    pose.orientation.y = qR[2];
    pose.orientation.z = qR[3]; 

    //ROS_INFO("nQuat: [w=%f,x=%f,y=%f,z=%f]",qR[0],qR[1],qR[2],qR[3]);  
    //ROS_INFO("Norm: %f",sqrt(qR[0]*qR[0]+qR[1]*qR[1]+qR[2]*qR[2]+qR[3]*qR[3]));   
}

void modifyPosition(float p[]){
    float pM[3];
    pM[0]=p[0]+(float)(randomDouble(-positionRange,positionRange)/100.0);
    pM[1]=p[1]+(float)(randomDouble(-positionRange,positionRange)/100.0);
    pM[2]=p[2]+(float)(randomDouble(-positionRange,positionRange)/100.0);    

    pose.position.x=pM[0];
    pose.position.y=pM[1];
    pose.position.z=pM[2];
    
    //ROS_INFO("nPos: [x=%f,y=%f,z=%f]",pM[0],pM[1],pM[2]);  
}

bool moveArm(){
    ROS_INFO("Move arm");
    bool success=true;

    moveit::planning_interface::MoveGroup group("right_arm");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setGoalTolerance(0.05);
    group.setPlanningTime(2.5);
    group.setStartState(*group.getCurrentState());

    //ROS_INFO("Reference frame: -%s-", group.getPlanningFrame().c_str());
    //ROS_INFO("End Effector Link: -%s-", group.getEndEffectorLink().c_str());
    //ROS_INFO("Position(HEAD) x:%f, y:%f, z:%f",fixationX,fixationY,fixationZ); 
    //ROS_INFO("Pos [x:%f, y:%f, z:%f]",pose.position.x,pose.position.y,pose.position.z); 

    float q[4];
    q[0]=pose.orientation.w;
    q[1]=pose.orientation.x;
    q[2]=pose.orientation.y;
    q[3]=pose.orientation.z;
    //ROS_INFO("Quat: [w=%f,x=%f,y=%f,z=%f]",q[0],q[1],q[2],q[3]); 

    float p[3]={pose.position.x,pose.position.y,pose.position.z};

    int maxIterations=20,iteration=0;
    do{
       success = group.setPoseTarget(pose,"r_palm_link");
       ROS_INFO("Set Position Target: %s", success? "SUCCESS" : "FAILED");
       if(success){
           ros::AsyncSpinner spinner(1);
           spinner.start();
           moveit::planning_interface::MoveGroup::Plan my_plan;
           success = group.plan(my_plan);
           ROS_INFO("Set Plan: %s", success? "SUCCESS" : "FAILED");
           if(success){
               int32_t error = moveit::planning_interface::MoveItErrorCode::FAILURE;
               error=group.move();
               if(error!= moveit::planning_interface::MoveItErrorCode::SUCCESS){
                   success=false;
                   ROS_INFO("Motion FAILED");
               }
           }
       }
       if(!success && iteration<maxIterations){
           iteration++;
           printf("\n");
           ROS_INFO("Iteration:%d",iteration); 
           modifyOrientation(q);
           modifyPosition(p);
       }
       else  
           ROS_INFO("Move Arm: SUCCESS");
    }while(!success && iteration<maxIterations);

    ROS_INFO("Move arm done");
    return success;
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

    servoing=true;
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

void callbackSavIm(const std_msgs::Bool::ConstPtr& msg){
    ROS_INFO("Save Image command received:");
    bool savImReceived = (bool) msg->data;   
    servoing=false;
}

void callbackVoice(const std_msgs::Int8::ConstPtr& msg){
    ROS_INFO("Voice command received");
    int voiceCMD=(int) msg->data;
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

double absolute(double in){
    double out=in;
    if(in<0)
       out=-in;
    return out;
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

void adjustHeadPosition(void){
    bool centered_polar=adjustHeadPositionPolar();
    bool centered_azimut=adjustHeadPositionAzimut();
    bool centered_radius=false;
    if(centered_azimut)
        centered_radius=adjustHeadPositionRadius();
    centered_complete=centered_polar && centered_azimut && centered_radius;
    fixationX= sphere_radius*sin(sphere_polar)*cos(sphere_azimut);
    fixationY=-sphere_radius*cos(sphere_polar);
    fixationZ= sphere_radius*sin(sphere_polar)*sin(sphere_azimut);
    moveHead(fixationX,fixationY,fixationZ);
}

void reachObject(){
    bool success;
    /*float p[3]={pose.position.x,pose.position.y,pose.position.z};
    float q[4]={pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w};

    //ROS_INFO("Quat: [w=%f,x=%f,y=%f,z=%f]",q[0],q[1],q[2],q[3]); 
    angleRange=15;
    positionRange=5;
    pose.position.x =  0.2;
    pose.position.y = -0.14;
    pose.position.z =  0.8;
    pose.orientation.x= 0.068340;
    pose.orientation.y=-0.564690;
    pose.orientation.z=-0.098815;
    pose.orientation.w= 0.816511;
    success=moveArm();

    pose.position.x = p[0];
    pose.position.y = p[1];
    pose.position.z = p[2];
    pose.orientation.x=q[0];
    pose.orientation.y=q[1];
    pose.orientation.z=q[2];
    pose.orientation.w=q[3];
    angleRange=5;
    positionRange=2.5;
    if(success)*/
        success=moveArm();
 /*   if(success)
        ROS_INFO("Object Reached");
    else
        ROS_INFO("Can not reach object");*/
}

int main(int argc, char **argv){
    srand (time(NULL));
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
     //Create trackbars in "Control" window
    cv::createTrackbar("LowH"  , OPENCV_WINDOW_COLOR_CONTROLS, &iLowH  , 179); //Hue (0 - 179)
    cv::createTrackbar("HighH" , OPENCV_WINDOW_COLOR_CONTROLS, &iHighH , 179);
    cv::createTrackbar("LowS"  , OPENCV_WINDOW_COLOR_CONTROLS, &iLowS  , 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS" , OPENCV_WINDOW_COLOR_CONTROLS, &iHighS , 255);
    cv::createTrackbar("LowV"  , OPENCV_WINDOW_COLOR_CONTROLS, &iLowV  , 255); //Value (0 - 255)
    cv::createTrackbar("HighV" , OPENCV_WINDOW_COLOR_CONTROLS, &iHighV , 255);
    cv::createTrackbar("Erode" , OPENCV_WINDOW_COLOR_CONTROLS, &iErode , 6  );
    cv::createTrackbar("Dilate", OPENCV_WINDOW_COLOR_CONTROLS, &iDilate, 6  );
    ROS_INFO("Grasp Learning");    
    ros::Rate loop_rate(10);

    tf::TransformListener listener(ros::Duration(1));
    ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener)));

     while (ros::ok()){ 
        if(centered_complete){
            servoing=false;
            visible_L=false;
            visible_R=false;
            centered_complete=false;   
            reachObject();
        }
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
