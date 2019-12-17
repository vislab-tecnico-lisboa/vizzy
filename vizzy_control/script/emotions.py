#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import *
import vizzy_msgs.msg
from std_msgs.msg import Int16, Float64
from time import sleep

class Emotions:
    leftArmPub = rospy.Publisher('/vizzyArmRoutines/left/command', Int16, queue_size=1)
    rightArmPub = rospy.Publisher('/vizzyArmRoutines/right/command', Int16, queue_size=1)
    torsoPub = rospy.Publisher('/vizzy/torso_joint/cmd', Float64, queue_size=1)

    def __init__(self):
	
        rospy.init_node('emotions', anonymous=False)
        rospy.on_shutdown(self.shutdown)

	rate = rospy.Rate(5) # 5hz

        self.gaze_client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)
        self.gaze_client.wait_for_server()
        rospy.loginfo("Connected to gaze server")

        while not rospy.is_shutdown():
	    self.home_pose()
	    self.sad_emotion_more_move()
	
	    self.home_pose()
	    self.happy_emotion_more_move()

	    self.home_pose()
            self.angry_emotion()

	    self.home_pose()
            self.fear_emotion()

	    self.home_pose()
	    self.surprise_emotion()

	    sleep(10)
            
    
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

    def home_pose(self):
	armL = 6
	armR = 6
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome() 
	sleep(4)


    def sad_emotion_more_move(self):
	armL = 9
	armR = 9
	torsoRad = 0.3
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeGoal(0.5, 0.0, 0.0)
	sleep(0.8)
	for i in range(5):
	    self.sendGazeGoal(0.5, 0.05, 0.0)
	    sleep(0.8)
	    self.sendGazeGoal(0.5, -0.05, 0.0)
	    sleep(0.8)
	self.sendGazeGoal(0.5, 0.0, 0.0)
	sleep(4)


    def sad_emotion_less_move(self):
	armL = 9
	armR = 9
	torsoRad = 0.3
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeGoal(0.5, 0.0, 0.0)
	sleep(4)


    def happy_emotion_more_move(self):
	armL = 8
	armR = 8
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()  
	sleep(10) # time to finish pose


    def happy_emotion_less_move(self):
	armL = 7
	armR = 7
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()  
	sleep(5)
	
    def angry_emotion(self):
	armL = 10
	armR = 10
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()  
	sleep(5)

    def fear_emotion(self):
	armL = 12
	armR = 11
	torsoRad = -0.1
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()  
	sleep(5)

    def surprise_emotion(self):
	armL = 13
	armR = 13
	torsoRad = 0.0
	self.leftArmPub.publish(armL)
	self.rightArmPub.publish(armR)
	self.torsoPub.publish(torsoRad)
	self.sendGazeHome()  
	sleep(5)
						
    
    def shutdown(self):
        rospy.loginfo("Shutting emotions script")

        
if __name__ == '__main__':
    try:
        Emotions()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gaze test done")


