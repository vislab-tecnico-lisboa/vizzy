#!/usr/bin/env python2
# -*- coding: utf-8 -*-

##########################################################################################
#THIS NODE IS EXPERIMENTAL. AFTER WE VALIDADE THAT IT WORKS OK, WE NEED TO REIMPLEMENT IT
#AS A C++ NODELET TO AVOID LATENCIES
##########################################################################################

import rospy

import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Twist
import cv2


class SaccadicSuppressor:
    def __init__(self):
        rospy.init_node('saccaddic_suppressor', anonymous=True)


        self.image_pub = rospy.Publisher("/supressed/compressed", \
                    CompressedImage, queue_size=1)
        
        self.image_sub = rospy.Subscriber("/image_in/compressed", CompressedImage, self.camera_callback,
        buff_size=2**24, queue_size=1)

        

    def camera_callback(self, image_compressed):

        np_arr = np.fromstring(image_compressed.data, np.uint8)
        last_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(last_img, cv2.COLOR_BGR2GRAY)

        fm = cv2.Laplacian(gray, cv2.CV_64F).var()

        print(fm)

        self.image_pub(image_compressed)


if __name__ == "__main__":
    try:
        SaccadicSuppressor()
    except rospy.ROSInterruptException:
        print("Shutting down 3d human detector")