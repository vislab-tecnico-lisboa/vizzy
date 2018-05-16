#!/usr/bin/env python

#    Copyright (C) 2016  Kriegel Joffrey
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
# Import required Python code.
import time
import roslib
import rospy
from std_msgs.msg import String
import sys

# Node example class.
class SlackTest():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
	
        # Create a publisher for our custom message.
        self.pub = rospy.Publisher('from_ros_to_slack', String, queue_size=10)
	rospy.Subscriber("from_slack_to_ros", String, self.callback)

        # Main while loop.
        while not rospy.is_shutdown():
	    # Sleep for a while before publishing new messages. Division is so rate != period.
            rospy.sleep(5.0)

    def callback(self, data):
        self.pub.publish("You just say : " + data.data)
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", data.data, self.channel)


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('slack_test')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        st = SlackTest()
    except rospy.ROSInterruptException: pass
