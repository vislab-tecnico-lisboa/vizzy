#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import math
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs

if __name__ == '__main__':
    rospy.init_node('tf2_end_effector_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.pose.orientation.w = 1.0
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform('base_link','l_wrist_flection_link', rospy.Time())
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        

        print(pose_transformed)
        
        rate.sleep()
