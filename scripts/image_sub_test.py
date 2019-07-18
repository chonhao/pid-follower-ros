#!/usr/bin/env python

import rospy
import cv2
import argparse
import time
from pid_follower.msg import follower_err
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()

def imageCallback(data):
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)
	
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)
	

if __name__ == '__main__':
	try:
		rospy.Subscriber("/camera/rgb/image_raw", Image, imageCallback)
		rospy.init_node('image_sub_test_node', anonymous=True)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass