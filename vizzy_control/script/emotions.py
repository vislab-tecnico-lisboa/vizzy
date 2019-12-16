#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
import vizzy_msgs.msg
from std_msgs.msg import Int16, Float64
from time import sleep

class Emotions:
    def __init__(self):
	leftArmPub = rospy.Publisher('/vizzyArmRoutines/left/command', Int16, queue_size=10)
	rightArmPub = rospy.Publisher('/vizzyArmRoutines/right/command', Int16, queue_size=10)
	torsoPub = rospy.Publisher('/vizzy/torso_joint/cmd', Float64, queue_size=10)

        rospy.init_node('emotions', anonymous=False)
        rospy.on_shutdown(self.shutdown)

	rate = rospy.Rate(5) # 5hz

        self.gaze_client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)
        self.gaze_client.wait_for_server()
        rospy.loginfo("Connected to gaze server")

        while not rospy.is_shutdown():
	    # reset position
            self.sendGazeHome()
	    armL = 6
	    armR = 6
	    torsoRad = 0.0
	    leftArmPub.publish(armL)
	    rightArmPub.publish(armR)
	    torsoPub.publish(torsoRad)
	    sleep(5)

	    armL = 7
	    armR = 7
	    torsoRad = 0.0
	    leftArmPub.publish(armL)
	    rightArmPub.publish(armR)
	    torsoPub.publish(torsoRad)
	    self.sendGazeGoal(0.0, 0.0, 0.5) 
	    sleep(10)
	
	    rate.sleep()
            
    
    def sendGazeGoal(self, x, y, z):
	#Look at a predefined 3d point in space
        gaze_point = vizzy_msgs.msg.GazeGoal()
        gaze_point.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
        gaze_point.fixation_point_error_tolerance = 0.01

        #Define the points reference frame.
        ##base_footprint is approximately centered below the front laser,
        ##in the floor. You can check the frames in Rviz.
        ### x axis - forward
        ### y axis - left
        ### z axis - up
        gaze_point.fixation_point.header.frame_id='base_footprint'

        gaze_point.fixation_point.point.x = x
        gaze_point.fixation_point.point.y = y
        gaze_point.fixation_point.point.z = z

        self.gaze_client.send_goal(gaze_point)
	

    def sendGazeHome(self):
	#Home goal makes the robot look forward
        home_goal = vizzy_msgs.msg.GazeGoal()
        home_goal.type = vizzy_msgs.msg.GazeGoal.HOME
        self.gaze_client.send_goal(home_goal)

    
    def shutdown(self):
        rospy.loginfo("Shutting emotions script")

        
if __name__ == '__main__':
    try:
        Emotions()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gaze test done")


