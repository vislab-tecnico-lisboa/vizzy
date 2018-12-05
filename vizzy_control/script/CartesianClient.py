#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import vizzy_msgs.msg
import actionlib
from actionlib_msgs.msg import *
import sys


def main():


    if len(sys.argv) < 4:
        print('Usage: python2 CartesianClient.py [pos_x] [pos_y] [pos_z]')
        exit()


    pos_x = float(sys.argv[1])
    pos_y = float(sys.argv[2])
    pos_z = float(sys.argv[3])

    
    rospy.init_node('cartesian_client')
    client = actionlib.SimpleActionClient('/vizzy/left_arm_cartesian_controller/cartesian_action', vizzy_msgs.msg.CartesianAction)
    
    print('Waiting server...')
    client.wait_for_server()
    print('Found cartesian action server')

    goal = vizzy_msgs.msg.CartesianGoal()
    goal.type = vizzy_msgs.msg.CartesianGoal.CARTESIAN

    #Qual e o referencial?? Nao tem header!
    goal.end_effector_pose.position.x = pos_x
    goal.end_effector_pose.position.y = pos_y
    goal.end_effector_pose.position.z = pos_z
    goal.end_effector_pose.orientation.w = 1.0

    client.send_goal(goal)



    rospy.spin()	

if __name__ == '__main__':
    main()
