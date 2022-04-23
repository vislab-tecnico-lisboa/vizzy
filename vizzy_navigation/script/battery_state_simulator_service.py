#!/usr/bin/env python

import rospy
from vizzy_msgs.srv import BatteryState
from vizzy_msgs.srv import BatteryStateResponse
from vizzy_msgs.srv import BatteryStateRequest

battery_state = 1
battery_percentage = 100


def check_battery_srv(req):
    res = BatteryStateResponse()

    res.percentage = battery_percentage
    
    if battery_percentage < 27:
        res.battery_state = BatteryStateResponse.LOW_BATTERY
    elif battery_percentage != 100:
        res.battery_state = BatteryStateResponse.GOOD
    else:
        res.battery_state = BatteryStateResponse.CHARGED

    return res 


def main_loop():

    fs = 10
    rate = rospy.Rate(fs) # 10hz
    global battery_percentage

    s = rospy.Service('kokam_battery_state', BatteryState, check_battery_srv)

    while not rospy.is_shutdown():
        battery_percentage = int(raw_input("Enter new percentage: ")) 
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("battery_state_simulator")

    try:
        main_loop()
    except rospy.ROSInterruptException:
        print("Shutting down battery state simulator")