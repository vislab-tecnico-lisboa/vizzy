#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Int16
from pedestrian_detector.msg import BBList
from geometry_msgs.msg import Point
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import time
arm_publisher = None
current_track = -1
sent_already= False
soundhandle = None
def subs_callback(data):
	global current_track
	global sent_already
	global soundhandle
	voice = 'voice_kal_diphone'
	volume = 4.0
	for bb_ind in range(0,len(data.bbVector)):
		if data.bbVector[bb_ind].tracked:
			if current_track == -1:
				current_track = data.bbVector[bb_ind].id
				is_different = True
			else:
				if current_track != data.bbVector[bb_ind].id:
					current_track = data.bbVector[bb_ind].id
					is_different = True
					sent_already = False
				else:
					is_different = False
			#print str(current_track) + ' ' +str(is_different)
			Mypoint = Point()
			MyPoint = data.bbVector[bb_ind].person3dLocation
			distance = MyPoint.x*MyPoint.x+MyPoint.y*MyPoint.y
			#print distance
			if distance < 1.5*1.5:
				if not is_different and not sent_already:
					arm_publisher.publish(1)
					sent_already = True
					print 'Saying hello to person ' + str(data.bbVector[bb_ind].id)
					soundhandle.say('Hello I am vizzy',voice,volume)
					time.sleep(4)
				if is_different:
					arm_publisher.publish(1)
					sent_already = True
					print 'Saying hello to person ' + str(data.bbVector[bb_ind].id)
					soundhandle.say('Hello I am vizzy',voice,volume)
					time.sleep(4)
			break

def main_demo():
	rospy.init_node('demo_sao_roque',anonymous=True)
	tracker_subs = rospy.Subscriber('bbs_with_id', BBList, subs_callback)
	global arm_publisher
	global soundhandle
	soundhandle = SoundClient()
	arm_publisher = rospy.Publisher('vizzyArmRoutines/command',Int16,queue_size=100)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
	    rate.sleep()

if __name__ == '__main__':
    try:
        main_demo()
    except rospy.ROSInterruptException:
        pass
