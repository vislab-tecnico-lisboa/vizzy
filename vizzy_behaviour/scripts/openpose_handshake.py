#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy

import vizzy_msgs.msg
import actionlib
from openpose_ros_msgs.msg import PersonDetection_3d, BodyPartDetection_3d

from std_msgs.msg import Int16

import time

from collections import deque

import numpy as np


class Handshaker():


    def __init__(self):

        rospy.init_node('openpose_handshaker')

	self.gazex = deque([0] * 5, maxlen=5)
	self.gazey = deque([0] * 5, maxlen=5)
	self.gazez = deque([0] * 5, maxlen=5)

	self.lastgazex = 0
	self.lastgazey = 0
	self.lastgazez = 0

        self.goal = vizzy_msgs.msg.GazeGoal()

        self.goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
        self.goal.fixation_point_error_tolerance = 0.01
        self.goal.fixation_point.header.frame_id='l_camera_vision_link'
        self.last_callback_time = rospy.get_time()
        self.last_handshake = rospy.get_time()
        self.last_wave = rospy.get_time()
        self.sm = StateMachine(self)

        self.rehablab_sub = rospy.Subscriber('/openpose_to_rehablab', PersonDetection_3d, self.callback)
        self.gesture_pub = rospy.Publisher('/vizzyArmRoutines/command', Int16, queue_size=1)
        self.client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)
        self.client.wait_for_server()
	self.vectorWaves = deque([0] * 35, maxlen=35)
	self.vectorHandshakes = deque([0] * 35, maxlen=35)



    def callback(self, person):

        if len(person.body_part) < 9:
            return

	
	self.gazex.append(person.body_part[0].x)
	self.gazey.append(person.body_part[0].y)
	self.gazez.append(person.body_part[0].z)

        #self.goal.fixation_point.header.stamp=rospy.get_rostime()
        self.goal.fixation_point.header = person.header
        self.goal.fixation_point.point.x = np.median(self.gazex)
        self.goal.fixation_point.point.y = np.median(self.gazey)
        self.goal.fixation_point.point.z = np.median(self.gazez)

	gazeVector = np.array([self.goal.fixation_point.point.x-self.lastgazex, self.goal.fixation_point.point.y-self.lastgazey, self.goal.fixation_point.point.z-self.lastgazez])

	if np.linalg.norm(gazeVector) > 0.2:
	    self.goal.fixation_point.point.x = self.lastgazex + np.sign(self.goal.fixation_point.point.x-self.lastgazex)*0.2
	    self.goal.fixation_point.point.y = self.lastgazey + np.sign(self.goal.fixation_point.point.y-self.lastgazey)*0.2
	    self.goal.fixation_point.point.z = self.lastgazez + np.sign(self.goal.fixation_point.point.z-self.lastgazez)*0.2
	
	self.lastgazex = self.goal.fixation_point.point.x
	self.lastgazey = self.goal.fixation_point.point.y
	self.lastgazez = self.goal.fixation_point.point.z


        self.last_callback_time = rospy.get_time()

        self.client.send_goal(self.goal)

	

        #detect a waving gesture, but only if the last one happened some time ago
        if (person.body_part[4].confidence > 0.3) and (person.body_part[4].y < person.body_part[0].y) and (rospy.get_time() - self.sm._time_since_last_gesture > 10):
            #self.sm.event_handler(self.sm._WAVING)
	    self.vectorWaves.append(1)

	if (person.body_part[7].confidence > 0.3) and (person.body_part[7].y < person.body_part[0].y) and (rospy.get_time() - self.sm._time_since_last_gesture > 10):
            #self.sm.event_handler(self.sm._WAVING)
	    self.vectorWaves.append(1)

	#detect handshake intention with depth 5 tree xD:
	if (person.body_part[4].confidence > 0.3) and (person.body_part[8].confidence > 0.3) and (person.body_part[7].confidence > 0.3) and (rospy.get_time() - self.sm._time_since_last_gesture > 20):
	    if person.body_part[4].x < person.body_part[7].x:
		if person.body_part[4].x > person.body_part[8].x:
		    if (person.body_part[4].y > person.body_part[1].y + (person.body_part[8].y-person.body_part[1].y)/3.5):
			if person.body_part[4].x > person.body_part[8].x:
			    if person.body_part[11].x - person.body_part[8].x > (person.body_part[1].y-person.body_part[0].y)*0.5:
			    #self.sm.event_handler(self.sm._HANDSHAKING)
			        self.vectorHandshakes.append(1)


    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            
            if rospy.get_time() - self.last_callback_time > 5: #if no detection after 5 seconds look in front
                goal = vizzy_msgs.msg.GazeGoal()
                goal.type = vizzy_msgs.msg.GazeGoal.HOME
                self.client.send_goal(goal)
            
	    self.sm.state_execution()
            rate.sleep()


class StateMachine():
    def __init__(self, handshaker_interface):
        self._handshaker_interface = handshaker_interface
        self._JUSTGAZING = 0
        self._HANDSHAKING = 1
        self._WAVING = 2
        self._state = self._JUSTGAZING
        self._time_since_last_gesture = 0

    def state_execution(self):
	if self._state == self._JUSTGAZING:
	    if np.median(self._handshaker_interface.vectorWaves) > 0.0:
		self.event_handler(self._WAVING)
		self._handshaker_interface.vectorWaves = deque([0] * 35, maxlen=35)
	    elif np.median(self._handshaker_interface.vectorHandshakes) > 0.0:
		self.event_handler(self._HANDSHAKING)
		self._handshaker_interface.vectorHandshakes = deque([0] * 35, maxlen=35)

        if self._state == self._WAVING:
            gesture_msg = Int16()
            gesture_msg.data = 1
            self._handshaker_interface.gesture_pub.publish(gesture_msg)
            self._time_since_last_gesture = rospy.get_time()
            self._state = self._JUSTGAZING
	    
	
	elif self._state == self._HANDSHAKING:
	    gesture_msg = Int16()
	    gesture_msg.data = 2
	    self._handshaker_interface.gesture_pub.publish(gesture_msg)
	    time.sleep(1.0)
	    gesture_msg = Int16()
	    gesture_msg.data = 3
	    self._handshaker_interface.gesture_pub.publish(gesture_msg)
	    self._time_since_last_gesture = rospy.get_time()
	    self._state = self._JUSTGAZING

    def event_handler(self, objective):
        if objective == self._WAVING:
            self._state = self._WAVING
	
	if objective == self._HANDSHAKING:
	    self._state = self._HANDSHAKING
      


if __name__ == '__main__':

    try:
        handshaker = Handshaker()
        handshaker.run()
    except rospy.ROSInterruptException:
        pass
