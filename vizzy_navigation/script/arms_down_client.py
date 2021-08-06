#!/usr/bin/env python

import roslib
import rospy
import sys
from vizzy_msgs.srv import ArmDown,ArmDownResponse,ArmDownRequest

def arm_down_client(my_request):
    rospy.wait_for_service('armDown')
    try:
        service_object = rospy.ServiceProxy('armDown',ArmDown)
        my_response = service_object(my_request)
        if (my_response.arm_down_reply==1):
            print('Success putting arm down')
        else:
            print('Unsuccessful putting arm down')
        return my_response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 3:
         my_arm_value = sys.argv[1]
         my_robot_type_value = sys.argv[2]
    my_request = ArmDownRequest()
    if my_arm_value=='left':
        my_request.arm_down_request = 0
    elif my_arm_value == "right":
        my_request.arm_down_request =1
    if my_robot_type_value == 'sim':
        my_request.robot_type = 1
    elif my_robot_type_value == 'real':
        my_request.robot_type = 0
    print("Requesting %s arm down in %s"%(my_arm_value,my_robot_type_value))
    print("%s"%(arm_down_client(my_request)))
