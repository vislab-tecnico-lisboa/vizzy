#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import vizzy_msgs.msg
from vizzy_msgs.msg import ChargeAction, ChargeGoal, ChargeFeedback, ChargeResult
import actionlib
from actionlib_msgs.msg import *
import sys
import argparse

def main():

    rospy.init_node('docking_client')
    client = actionlib.SimpleActionClient('charging_action', ChargeAction)
    
    print('Waiting server...')
    client.wait_for_server()
    print('Found docking action server')

    goal = ChargeGoal()
    goal.goal = goal.STOP_CHARGE
    client.send_goal(goal)

    client.wait_for_result()
    print client.get_result()
    rospy.spin()

if __name__ == '__main__':
    main()
