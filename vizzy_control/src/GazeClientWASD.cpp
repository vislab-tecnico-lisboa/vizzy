/* Joao Avelino. Based on GazeClient.cpp by Rui Figueiredo
*/

#include "GazeClientWASD.hpp"
#include <unistd.h>
#include <termios.h>


int main (int argc, char **argv)
{
     
    /*Stuff to get keys without pressing enter*/
    struct termios old_tio, new_tio;
    unsigned char c;

    /* get the terminal settings for stdin */
    tcgetattr(STDIN_FILENO,&old_tio);

    /* we want to keep the old setting to restore them a the end */
    new_tio=old_tio;

    /* disable canonical mode (buffered i/o) and local echo */
    new_tio.c_lflag &=(~ICANON & ~ECHO);

    /* set the new settings immediately */
    tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

    ros::init(argc, argv, "gazewasd");

    ros::NodeHandle nh;
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<vizzy_msgs::GazeAction> ac("gaze", true);

    ROS_INFO("Waiting for action server to start.");
    ROS_INFO("Use W, A, S, D like a real gamer to control Vizzy's head. W, S -> controls z values; A, D controls y values. C, V controls x values");
    ROS_INFO("Use I, J, K, L for coarse control. I, K -> controls z values; J, L controls y values. N, M controls x values");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    double rate=50.0;
    ros::Rate r(rate);

    double step_size=0.1/10;

    // send a goal to the action
    vizzy_msgs::GazeGoal goal;
    goal.type=vizzy_msgs::GazeGoal::CARTESIAN;
    goal.fixation_point.point.x = 2;
    goal.fixation_point.point.y = 0;
    goal.fixation_point.point.z = 0;
    goal.fixation_point_error_tolerance = 0.01;

    while(nh.ok())
    {

        goal.fixation_point.header.frame_id="ego_frame";
        goal.fixation_point.header.stamp=ros::Time::now();

	/*Teleop - W, A, S, D, C, V*/
    
        int c = getchar();

        const float step = 0.05;

        if(c == 'W' || c == 'w')
        goal.fixation_point.point.z += 5*step;
        if(c == 'S' || c == 's')
        goal.fixation_point.point.z -= 5*step;
        if(c == 'A' || c == 'a')
        goal.fixation_point.point.y += 5*step;
        if(c == 'D' || c == 'd')
        goal.fixation_point.point.y -= 5*step;
        if(c == 'C' || c == 'c')
        goal.fixation_point.point.x += 5*step;
        if(c == 'I' || c == 'i')
        goal.fixation_point.point.z += 10*step;
        if(c == 'K' || c == 'k')
        goal.fixation_point.point.z -= 10*step;
        if(c == 'J' || c == 'j')
        goal.fixation_point.point.y += 10*step;
        if(c == 'L' || c == 'l')
        goal.fixation_point.point.y -= 10*step;
        if(c == 'N' || c == 'n')
        goal.fixation_point.point.x += 10*step;
        if(c == 'M' || c == 'm')
        goal.fixation_point.point.x -= step;

	  

	    goal.fixation_point.header.frame_id="camera_link";
        goal.fixation_point.header.stamp=ros::Time::now();
    

        ac.sendGoal(goal);
        ROS_INFO("Action server started, sending gaze goal.");

        r.sleep();

    }
    
    /* restore the former settings */
    tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);
    //exit
    return 0;
}
