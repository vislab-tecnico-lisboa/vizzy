#!/usr/bin/env python


# Import required Python code.
import time
import roslib
import rospy
from std_msgs.msg import String
import sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Node example class.
class SlackVizzyBot():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Create a publisher for our custom message.
        rospy.Subscriber("/vizzy/slack/from_slack_to_ros", String, self.callback)
        self.soundhandle = SoundClient()

        # Main while loop.
        while not rospy.is_shutdown():
        # Sleep for a while before publishing new messages. Division is so rate != period.
            rospy.sleep(5.0)


    def callback(self, data):
        #self.pub.publish("You just say : " + data.data)
        message_split = data.data.split()
        if message_split[0] == 'vizzy':
            if message_split[1] == 'say':
                voice = 'voice_kal_diphone'
                self.soundhandle.say(' '.join(message_split[2:]),voice,4)
            if message_split[1] == 'navigate' and message_split[2] == 'to':
                destination = " ".join(message_split[3:])
                print("I want to go to " + destination)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('slack_vizzy_bot')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        st = SlackVizzyBot()
    except rospy.ROSInterruptException: pass
