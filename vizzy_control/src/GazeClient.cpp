#include "GazeClient.h"
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

    ros::init(argc, argv, "test_gaze");

    ros::NodeHandle nh;
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<vizzy_msgs::GazeAction> ac("gaze", true);

    ROS_INFO("Waiting for action server to start.");
    ROS_INFO("Use W, A, S, D like a real gamer to control Vizzy's head. W, S -> controls z values; A, D controls y values. C, V controls x values");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    double rate=2.0;
    ros::Rate r(rate);

    double rate_aux=0.1;

    // send a goal to the action
    int i=0;
    vizzy_msgs::GazeGoal goal;
    goal.type=vizzy_msgs::GazeGoal::CARTESIAN;
    goal.fixation_point.point.x = 2;
    goal.fixation_point.point.y = 0;
    goal.fixation_point.point.z = 1;
    goal.fixation_point_error_tolerance = 0.01;

    while(nh.ok())
    {

        ++i;

        /*double angular_freq=2*M_PI*rate_aux;
        double time_instant=(double)i;
        double aux=angular_freq*time_instant;
        std::cout << aux << " "<<cos(aux) <<std::endl;
        vizzy_msgs::GazeGoal goal;
        goal.type=vizzy_msgs::GazeGoal::CARTESIAN;
        goal.fixation_point.point.x = 0.7*cos(aux);
        goal.fixation_point.point.y = 0.0;
        goal.fixation_point.point.z = 10.0+9.0*cos(aux);
        goal.fixation_point_error_tolerance = 0.005;

        //goal.fixation_point.point.z = 0.5;

        goal.fixation_point.header.frame_id="ego_frame";
        goal.fixation_point.header.stamp=ros::Time::now();*/

    /*AVELINO*/
	/*Teleop - W, A, S, D, C, V*/

    int c = getchar();

	const float step = 0.05;

	if(c == 'W' || c == 'w')
	  goal.fixation_point.point.z += step;
	if(c == 'S' || c == 's')
	  goal.fixation_point.point.z -= step;
	if(c == 'A' || c == 'a')
	  goal.fixation_point.point.y += step;
	if(c == 'D' || c == 'd')
	  goal.fixation_point.point.y -= step;
	if(c == 'C' || c == 'c')
	  goal.fixation_point.point.x += step;
	if(c == 'V' || c == 'v')
	  goal.fixation_point.point.x -= step;
	  

	goal.fixation_point.header.frame_id="base_footprint";
        goal.fixation_point.header.stamp=ros::Time::now();
    /*FIM AVELINO*/

        ac.sendGoal(goal);
        ROS_INFO("Action server started, sending goal.");

        //wait for the action to return
        //bool finished_before_timeout = ac.waitForResult(ros::Duration(1.0));

        /*if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
            ROS_INFO("Action did not finish before the time out.");*/

        r.sleep();

    }
    
    /* restore the former settings */
    tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);
    //exit
    return 0;
}
