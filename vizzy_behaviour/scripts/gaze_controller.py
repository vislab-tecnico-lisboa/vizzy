#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import vizzy_msgs.msg
from std_msgs.msg import Int16
import actionlib

from actionlib_msgs.msg import *


import threading

from OpenFace.msg import intent_msg
from OpenFace.msg import intent_msg_all



index_gaze = -1



class person_intent:
 def __init__(self,px,py,pz,l,g,ri,bh,bw,bx,by,id):

    self.pose_tra_x =px
    self.pose_tra_y =py
    self.pose_tra_z =pz
    self.looking =l
    self.gesture =g
    self.result_interact =ri
    self.box_h =bh
    self.box_w =bw
    self.box_x =bx
    self.box_y =by
    self.id_model =id




def callback(data):


	list_of_persons=[]
	global index_gaze
	for i in range(data.total_models):
	    new_person= person_intent(data.intent_person[i].pose_tra_x,data.intent_person[i].pose_tra_y,data.intent_person[i].pose_tra_z,data.intent_person[i].looking,data.intent_person[i].gesture,data.intent_person[i].result_interact,int(data.intent_person[i].box_h),int(data.intent_person[i].box_w),int(data.intent_person[i].box_x),int(data.intent_person[i].box_y),data.intent_person[i].id_model)
	    list_of_persons.append(new_person)
	lost = 1
        for person in list_of_persons:
	    if(person.id_model==index_gaze):
		    lost=0			
		    gazeclient(person.pose_tra_x,person.pose_tra_y,person.pose_tra_z)
		    rospy.sleep(0.05)
	if(lost == 1):
	    gazeclient(0,0,1)


def callback_index(data):
	global index_gaze
	index_gaze=data.data


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
	goal.fixation_point.point.x = z
	goal.fixation_point.point.y = x
	goal.fixation_point.point.z = y+0.3
	goal.fixation_point.header.frame_id='camera_link'
	goal.fixation_point.header.stamp=rospy.get_rostime()

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	#client.wait_for_result()

	return client.get_result()

	
# main
def main():
    
    

    rospy.init_node('gaze_controller')
    
   
    intent_sub = rospy.Subscriber('continuos_intent_detector_win',intent_msg_all,callback)
    subscribe_index = rospy.Subscriber('gaze_index',Int16,callback_index)



    rospy.spin()	

if __name__ == '__main__':
    main()




