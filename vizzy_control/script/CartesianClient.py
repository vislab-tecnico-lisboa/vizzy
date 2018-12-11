#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import vizzy_msgs.msg
import actionlib
from actionlib_msgs.msg import *
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description='Calls the cartesian action client for Vizzy.')
    parser.add_argument('type', type=int, help='Type of action to be executed, 0-Cartesian point, 1-Home position, 2-Cartesian velocity, 3-Close hand, 4-Open hand')
    parser.add_argument('type_args', type=float, nargs='*', \
        help='If 0, you can send either the cartesian position [pos x] [pos y] [pos z] or the position and orientation [pos x] [pos y] [pos z] [quat x] [quat y] [quat z] [quat w]')
    my_namespace=parser.parse_args()
    print my_namespace.type_args

    if my_namespace.type==0 and len(my_namespace.type_args) < 3:
        print('Usage: python2 CartesianClient.py 0 [pos x] [pos y] [pos z]')
        exit()
    elif my_namespace.type==0 and len(my_namespace.type_args) > 3 and len(my_namespace.type_args) != 7:
        print('Usage: python2 CartesianClient.py 0 [pos x] [pos y] [pos z] [quat x] [quat y] [quat z] [quat w]')
        exit()
    elif my_namespace.type==1 and len(my_namespace.type_args)!=0:
        print('Usage: python2 CartesianClient.py 1')
        exit()
    elif my_namespace.type==2 and len(my_namespace.type_args)!=4:
        print('Usage: python2 CartesianClient.py 2 [vel x] [vel y] [vel z] [time t]')
        exit()
    elif my_namespace.type == 3 and len(my_namespace.type_args)!=0:
        print('Usage: python2 CartesianClient.py 3')
        exit()
    elif my_namespace.type == 4 and len(my_namespace.type_args)!=0:
        print('Usage: python2 CartesianClient.py 4')
        exit()
    elif my_namespace.type == 5 and len(my_namespace.type_args)!=0:
        print('Usage: python2 CartesianClient.py 5')
        exit()

    goal = vizzy_msgs.msg.CartesianGoal()
    if my_namespace.type == 0:
        goal.type = vizzy_msgs.msg.CartesianGoal.CARTESIAN
        goal.end_effector_pose.position.x = my_namespace.type_args[0]
        goal.end_effector_pose.position.y = my_namespace.type_args[1]
        goal.end_effector_pose.position.z = my_namespace.type_args[2]
        if len(my_namespace.type_args) > 3:
            goal.end_effector_pose.orientation.x = my_namespace.type_args[3]
            goal.end_effector_pose.orientation.y = my_namespace.type_args[4]
            goal.end_effector_pose.orientation.z = my_namespace.type_args[5]
            goal.end_effector_pose.orientation.w = my_namespace.type_args[6]
        else:
            goal.end_effector_pose.orientation.x = 0
            goal.end_effector_pose.orientation.y =0
            goal.end_effector_pose.orientation.z = 0
            goal.end_effector_pose.orientation.w = 1
    elif my_namespace.type==1:
        goal.type = vizzy_msgs.msg.CartesianGoal.HOME
    elif my_namespace.type==2:
        goal.type = vizzy_msgs.msg.CartesianGoal.VELOCITY
        my_val=std_msgs.msg.Float32()
        my_val.data = my_namespace.type_args[0]
        goal.velocity.append(my_val)
        my_val1=std_msgs.msg.Float32()
        my_val1.data = my_namespace.type_args[1]
        goal.velocity.append(my_val1)
        my_val2=std_msgs.msg.Float32()
        my_val2.data = my_namespace.type_args[2]
        goal.velocity.append(my_val2)
        my_val3=std_msgs.msg.Float32()
        my_val3.data=my_namespace.type_args[3]
        goal.duration = my_val3
    elif my_namespace.type == 3:
        goal.type = vizzy_msgs.msg.CartesianGoal.GRAB
    elif my_namespace.type == 4:
        goal.type = vizzy_msgs.msg.CartesianGoal.RELEASE
    elif my_namespace.type == 5:
        goal.type = vizzy_msgs.msg.CartesianGoal.PREEMPT

    
    rospy.init_node('cartesian_client')
    client = actionlib.SimpleActionClient('/vizzy/left_arm_cartesian_controller/cartesian_action', vizzy_msgs.msg.CartesianAction)
    
    print('Waiting server...')
    client.wait_for_server()
    print('Found cartesian action server')

    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()



    rospy.spin()	

if __name__ == '__main__':
    main()
