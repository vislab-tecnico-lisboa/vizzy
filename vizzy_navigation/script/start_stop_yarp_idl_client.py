#!/usr/bin/env python

import roslib
import rospy
import sys
from vizzy_msgs.srv import ShutdownStartYarpRunProcess,ShutdownStartYarpRunProcessRequest,ShutdownStartYarpRunProcessResponse

def shutdown_start_client(my_request):
    rospy.wait_for_service('shutdownStartProcess')
    try:
        service_object = rospy.ServiceProxy('shutdownStartProcess',ShutdownStartYarpRunProcess)
        my_response = service_object(my_request)
        if (my_response.shutdown_reply==1):
            print('Success stopping process')
        else:
            print('Unsuccessful stopping process')
        return my_response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 2:
        my_request_value = sys.argv[1]
    my_request = ShutdownStartYarpRunProcessRequest()
    my_request.shutdown_request = int(my_request_value)
    my_request.server_port = '/vizzy-desktop'
    my_request.tag_str = ":vizzy-desktopyarpidl_rosmsg--name::typ@:yarpidl0"
    my_request.command_str = "yarpidl_rosmsg --name /typ@/yarpidl"
    if int(my_request_value)==0:
        my_str = "shutdown"
    else:
        my_str = "start"
    print("Requesting %s process command: %s"%(my_str,my_request.command_str))
    print("%s"%(shutdown_start_client(my_request)))
