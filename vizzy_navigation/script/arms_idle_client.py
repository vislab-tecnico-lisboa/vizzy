import roslib
import rospy
from vizzy_msgs.srv import MotorsShutdown,MotorsShutdownResponse,MotorsShutdownRequest

def idle_motors_client():
    rospy.wait_for_service('arm_motors_idle_server')
    try:
        service_object = rospy.ServiceProxy('arm_motors_idle_server',MotorsShutdown)
        my_request = MotorsShutdownRequest()
        my_request.shutdown_request = 0
        my_response = service_object(my_request)
        if (my_response.shutdown_reply==1):
            print('Success turning off the arms')
        else:
            print('Unsuccessful trying to setting arms idle')
        return my_response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Requesting setting arms idle")
    print("%s"%(idle_motors_client()))
