#!/user/bin/env python

import roslib
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actionlib

from actionlib_msgs.msg import *

import woz_dialog_msgs.msg
import vizzy_msgs.msg


# EXEMPLO ACTION FALA
def speak():


    client = actionlib.SimpleActionClient('/woz_dialog/speaker', woz_dialog_msgs.msg.SpeechAction)

    client.wait_for_server()

    goal = woz_dialog_msgs.msg.SpeechGoal(language="eng-USA", voice="Zoe", message="I am going to build a wall")

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result() 


# EXEMPLO ACTION GAZE
def gazeclient(x,y,z):
	# Creates the SimpleActionClient, passing the type of the action
	# (vizzy_msgs.msg.GazeAction) to the constructor.
	client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	goal = vizzy_msgs.msg.GazeGoal()
	goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
	goal.fixation_point_error_tolerance = 0.01
	goal.fixation_point.point.x = x
	goal.fixation_point.point.y = y
	goal.fixation_point.point.z = z
	goal.fixation_point.header.frame_id='base_footprint'
	goal.fixation_point.header.stamp=rospy.get_rostime()

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	#client.wait_for_result()

	return client.get_result()

# EXEMPLO ACTION NAVEGACAO NO MAPA
def navigate(x,y,w)
	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	move_base.wait_for_server(rospy.Duration(5))
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.orientation.w = w
	move_base.send_goal(goal)
	return move_base.wait_for_result(rospy.Duration(60))
