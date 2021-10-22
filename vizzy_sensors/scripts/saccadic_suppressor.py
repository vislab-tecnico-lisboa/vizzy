#!/usr/bin/env python2
# -*- coding: utf-8 -*-

##########################################################################################
#THIS NODE IS EXPERIMENTAL. AFTER WE VALIDADE THAT IT WORKS OK, WE NEED TO REIMPLEMENT IT
#IN THE C++ NODELET TO AVOID LATENCIES
##########################################################################################

import rospy

import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import woz_dialog_msgs.msg
import actionlib


import cv2


class SaccadicSuppressor:
    def __init__(self):
        rospy.init_node('saccaddic_suppressor', anonymous=True)


        self.image_pub = rospy.Publisher("/supressed/compressed", \
                    CompressedImage, queue_size=1)
        self.image_sub = rospy.Subscriber("/image_in/compressed", CompressedImage, self.camera_callback,
        buff_size=2**24, queue_size=1)

        self.client = actionlib.SimpleActionClient('/gcloud_tts', woz_dialog_msgs.msg.SpeechAction)

        self.laplacian_pub = rospy.Publisher("/laplacian", String, queue_size=1)

        self.lap_thr = rospy.get_param("~lap_thr", 50)
        #self.lap_thr = rospy.get_param("~lap_thr", 10)
        self.speech_thr = rospy.get_param("~speech_thr", 10)

        self.counter = 51

        rospy.spin()

    def camera_callback(self, image_compressed):

        np_arr = np.fromstring(image_compressed.data, np.uint8)
        last_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(last_img, cv2.COLOR_BGR2GRAY)

        fm = cv2.Laplacian(gray, cv2.CV_64F).var()

        self.counter += 1


        if fm < self.lap_thr:
            if self.counter >=50 and fm < self.speech_thr:
                goal_pt = woz_dialog_msgs.msg.SpeechGoal(language='pt_PT', \
                                                         voice='pt-PT-Wavenet-D',\
                                                         message='Não consigo ver',\
                                                         speed = 2)
                self.client.send_goal(goal_pt)
                self.counter = 0
            return

        lap = String()
        lap.data = str(fm)
        self.laplacian_pub.publish(lap)

        self.image_pub.publish(image_compressed)



if __name__ == "__main__":
    try:
        SaccadicSuppressor()
    except rospy.ROSInterruptException:
        print("Shutting Saccadic Suppressor")
