/*By Rui Figueiredo*/

#include "GazeClient.h"


int main (int argc, char **argv)
{

    ros::init(argc, argv, "test_gaze");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh_("~");
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<vizzy_msgs::GazeAction> ac("gaze", true);

    ac.waitForServer(); //will wait for infinite time



    double rate=50.0;

    priv_nh_.param("rate", rate, rate);
    ros::Rate r(rate);

    double step_size=0.1/10;





    // send a goal to the action
    int i=0;

    while(nh.ok())
    {

        ++i;

        double angular_freq=2*M_PI*step_size;
        double time_instant=(double)i;
        double aux=angular_freq*time_instant;
        std::cout << aux << " "<<cos(aux) <<std::endl;
        vizzy_msgs::GazeGoal goal;
        goal.type=vizzy_msgs::GazeGoal::CARTESIAN;
        goal.fixation_point.point.x = 3*cos(aux);
        goal.fixation_point.point.y = 0.0;
        goal.fixation_point.point.z = 5.0;//+9.0*cos(aux);
        goal.fixation_point_error_tolerance = 0.005;

        //goal.fixation_point.point.z = 0.5;

        goal.fixation_point.header.frame_id="ego_frame";
        goal.fixation_point.header.stamp=ros::Time::now();


        ac.sendGoal(goal);
        ROS_INFO("Action server started, sending gaze goal.");

        r.sleep();

    }
    
    /* restore the former settings */
    //exit
    return 0;
}
