#!/usr/bin/env python

import roslib
import rospy
import sys
from vizzy_msgs.srv import MotorsShutdown,MotorsShutdownResponse,MotorsShutdownRequest

def idle_motors_client(my_request):
    rospy.wait_for_service('armMotorsIdle')
    try:
        service_object = rospy.ServiceProxy('armMotorsIdle',MotorsShutdown)
        my_response = service_object(my_request)
        if (my_response.shutdown_reply==1):
            print('Success turning off the arms')
        else:
            print('Unsuccessful trying to setting arms idle')
        return my_response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 2:
        my_request_value = sys.argv[1]
        my_request = MotorsShutdownRequest()
    if my_request_value == 'start':
        my_request.shutdown_request = 1
    elif my_request_value == 'stop':
        my_request.shutdown_request = 0
    print("Requesting setting arms idle")
    print("%s"%(idle_motors_client(my_request)))
