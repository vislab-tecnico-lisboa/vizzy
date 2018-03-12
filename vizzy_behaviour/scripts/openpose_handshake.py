#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy

import vizzy_msgs.msg
import actionlib
from openpose_ros_msgs.msg import PersonDetection_3d, BodyPartDetection_3d


class Handshaker():

    def __init__(self):
        rospy.init_node('openpose_handshaker')
        self.rehablab_sub = rospy.Subscriber('/openpose_to_rehablab', PersonDetection_3d, self.callback)
        self.client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)
        self.client.wait_for_server()

        self.goal = vizzy_msgs.msg.GazeGoal()
        self.goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
        self.goal.fixation_point_error_tolerance = 0.01
        self.goal.fixation_point.header.frame_id='l_camera_vision_link'

        rospy.spin()

    def callback(self, person):

        if len(person.body_part) < 1:
            return
        
        #self.goal.fixation_point.header.stamp=rospy.get_rostime()
        self.goal.fixation_point.header = person.header
        self.goal.fixation_point.point.x = person.body_part[0].x
        self.goal.fixation_point.point.y = person.body_part[0].y
        self.goal.fixation_point.point.z = person.body_part[0].z

        self.client.send_goal(self.goal)
      


if __name__ == '__main__':

    try:
        handshaker = Handshaker()
    except rospy.ROSInterruptException:
        pass