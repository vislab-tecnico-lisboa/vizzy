#!/usr/bin/env python 

#Joao Avelino, Alexandra Goncalves 2020/2021
#ISR-Lisboa / IST

import random
import ast

#ROS imports
import rospy
import actionlib

#General action files
from vizzy_behavior_trees.msg import GeneralAction, GeneralFeedback, GeneralResult

#Laser scan messages
from sensor_msgs.msg import LaserScan

#Direct wheel command messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

#Odometry messages: wheel information from encoders (unreliable in real scenarios)
from nav_msgs.msg import Odometry

from locale import atof

import rospkg


class RandomMovementServer(object):
    # create messages that are used to publish feedback/result
    _feedback = GeneralFeedback()
    _result = GeneralResult()

    def __init__(self, name):

        #Choosing the desired coordinates

        pkg_path = rospkg.RosPack().get_path('vizzy_navigation') 

        coords_file = rospy.get_param("~coords_file", "coords-7th-floor-ISR.txt")

        self.frame_id = rospy.get_param("~frame_id", "map")

        self.txt=open(pkg_path+"/coords/"+coords_file,"r")
        self.limits=ast.literal_eval(self.txt.read())
        self.visited = '0'

        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, GeneralAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.spin()
      

    def execute_cb(self, goal):

        success=True

        a=0
        for i in goal.constants:
            a+=1
            if i==self.visited:
                break

        if a==len(goal.constants):
            a=0

        room_limits=self.limits[ast.literal_eval(goal.constants[a])]
	
        x=round(random.uniform(room_limits[0][0],room_limits[0][1]),4)
        y=round(random.uniform(room_limits[1][0],room_limits[1][1]),4)
        result=str(x)+";"+str(y)+";0;0;0;0;1"

        position = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=0.5)

        x1=position.pose.pose.position.x
        y1=position.pose.pose.position.y
 

        if self.limits[0][0][0] <= x1 <= self.limits[0][0][1] and self.limits[0][1][0] <= y1 <= self.limits[0][1][1]:
            self.visited='0'
        elif self.limits[1][0][0] <= x1 <= self.limits[1][0][1] and self.limits[1][1][0] <= y1 <= self.limits[1][1][1]:
            self.visited='1'
        elif self.limits[2][0][0] <= x1 <= self.limits[2][0][1] and self.limits[3][1][0] <= y1 <= self.limits[2][1][1]:
            self.visited='2'
        elif self.limits[3][0][0] <= x1 <= self.limits[3][0][1] and self.limits[3][1][0] <= y1 <= self.limits[3][1][1]:
            self.visited='3'

	    

	
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
                  
        if success:
            self._result.result = result
            self._result.header.stamp = rospy.get_rostime()
            self._result.header.frame_id = self.frame_id
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            
        else:
            self._as.set_aborted()
            self._result.result = "Nao consigo andar. Um obstaculo!"
            rospy.loginfo('%s: Failed' % self._action_name)


        
if __name__ == '__main__':
    rospy.init_node('random_movement')

    try:
        server = RandomMovementServer(rospy.get_name())
    except Exception as e:
        print("Shutting down random movement node: " + str(e))

