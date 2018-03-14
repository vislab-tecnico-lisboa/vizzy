#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy

import vizzy_msgs.msg
import actionlib
from openpose_ros_msgs.msg import PersonDetection_3d, BodyPartDetection_3d

from std_msgs.msg import Int16


class Handshaker():

    def __init__(self):
        rospy.init_node('openpose_handshaker')
        self.rehablab_sub = rospy.Subscriber('/openpose_to_rehablab', PersonDetection_3d, self.callback)
        self.gesture_pub = rospy.Publisher('/vizzyArmRoutines/command', Int16)
        self.client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)
        self.client.wait_for_server()

        self.goal = vizzy_msgs.msg.GazeGoal()
        self.goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
        self.goal.fixation_point_error_tolerance = 0.01
        self.goal.fixation_point.header.frame_id='l_camera_vision_link'
        self.last_callback_time = rospy.get_time()
        self.last_handshake = rospy.get_time()
        self.last_wave = rospy.get_time()
        self.sm = StateMachine(self)



    def callback(self, person):

        if len(person.body_part) < 1:
            return
        
        #self.goal.fixation_point.header.stamp=rospy.get_rostime()
        self.goal.fixation_point.header = person.header
        self.goal.fixation_point.point.x = person.body_part[0].x
        self.goal.fixation_point.point.y = person.body_part[0].y
        self.goal.fixation_point.point.z = person.body_part[0].z

        self.last_callback_time = rospy.get_time()

        self.client.send_goal(self.goal)

        #detect a waving gesture, but only if the last one happened some time ago
        if (person.body_part[4].confidence > 0.3) and (person.body_part[4].y < person.body_part[0].y) and (self.sm._time_since_last_gesture-rospy.get_time() > 10):
            self.sm.event_handler(self.sm._WAVING)


    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            
            if self.last_callback_time - rospy.get_time() > 10: #if no detection after 10 seconds look in front
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
        self._time_since_last_gesture = rospy.get_time()

    def state_execution(self):
        if self._state == self._WAVING:
            gesture_msg = Int16()
            gesture_msg.data = 1
            self._handshaker_interface.gesture_pub.publish(gesture_msg)
            self._time_since_last_gesture = rospy.get_time()
            self._state = self._JUSTGAZING


    def event_handler(self, objective):
        if objective == self._WAVING:
            self._state = self._WAVING
      


if __name__ == '__main__':

    try:
        handshaker = Handshaker()
        handshaker.run()
    except rospy.ROSInterruptException:
        pass