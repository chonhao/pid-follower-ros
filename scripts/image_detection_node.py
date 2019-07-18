#!/usr/bin/env python

import rospy
import cv2, sys, os
import argparse
import time
from pid_follower.msg import follower_err
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

global pub
global x, y, w, h
x = 0
y = 0
w = 0
h = 0
msg = follower_err()
tracker = None
# tracker = cv2.TrackerGOTURN_create()
initBB = None

ADJUST_PID = True

def imageCallback(data):
	#change ros image to opencv image
	try:
		bridge = CvBridge()
		imageFrame = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	#resize opencv image
	(W, H) = (500, 375)
	imageFrame = cv2.resize(imageFrame, (W,H), interpolation=cv2.INTER_CUBIC)

	if ADJUST_PID:
		initBB = (W-50, 375/2-50, 50, 100)
		tracker = cv2.TrackerMedianFlow_create()
		tracker.init(imageFrame, initBB)
		global ADJUST_PID
		ADJUST_PID = False

	#apply tracker
	global initBB
	msg.is_obj_being_tracked = 0
	if initBB is not None:
		# grab the new bounding box coordinates of the object
		global tracker
		(success, box) = tracker.update(imageFrame)

		# check to see if the tracking was a success
		if success:
			global x, y, w, h
			msg.is_obj_being_tracked = 1.0
			(x, y, w, h) = [int(v) for v in box]
			cv2.rectangle(imageFrame, (x, y), (x + w, y + h),
				(0, 255, 0), 2)

			# Calculating distance from center IN PIXEL
			FrameCenter = W/2
			BoxCenter = (int)(x+w/2)
			PixelFromCenter = BoxCenter - FrameCenter
			msg.direction_err = PixelFromCenter
		else:
			initBB = None
			msg.is_obj_being_tracked = 0
			msg.direction_err = 0.0

		# initialize the set of information we'll be displaying on
		# the frame
		info = [
			("Success", "Yes" if success else "No"),
			("Direction from center ",msg.direction_err),
			("Distance ", msg.distance_err)
			# ("Depth(mm)",msg.distance_err)
		]

		# loop over the info tuples and draw them on our frame
		for (i, (k, v)) in enumerate(info):
			text = "{}: {}".format(k, v)
			cv2.putText(imageFrame, text, (10, H - ((i * 20) + 20)),
				cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

	#show output frame
	cv2.imshow("Frame", imageFrame)
	key = cv2.waitKey(1) & 0xFF
	pub.publish(msg)

	#select tracking object
	if key == ord("s"):
		initBB = cv2.selectROI("Frame", imageFrame, fromCenter=False,
			showCrosshair=True)
		tracker = None
		tracker = cv2.TrackerMedianFlow_create()
		tracker.init(imageFrame, initBB)

def depthCallback(data):
	try:
		bridge = CvBridge()
		depthFrame = bridge.imgmsg_to_cv2(data, "passthrough")
		(W, H) = (500, 375)
		depthFrame = cv2.resize(depthFrame, (W,H), interpolation=cv2.INTER_CUBIC)
	except CvBridgeError as e:
		print(e)

	#get box center's depth data
	if msg.is_obj_being_tracked == 1:
		global x, y, w, h
		BoxCenterX = (x+(w/2))
		BoxCenterY = (y+(h/2))
		if depthFrame[BoxCenterY, BoxCenterX] > 600:
			msg.distance_err = depthFrame[BoxCenterY, BoxCenterX]
		else:
			msg.distance_err = -1

	#get minimum depth data in box
	# if msg.is_obj_being_tracked == 1:
	# 	global x, y, w, h
	# 	objectDepth = 8000
	# 	for i in range (x, x+w):
	# 		for j in range (y, y+h):
	# 			if (depthFrame[i, j] > 500 and depthFrame[i, j] < objectDepth):
	# 				objectDepth = depthFrame[i,j]
	# 	msg.distance_err = objectDepth

	#get average depth data in box
	# if msg.is_obj_being_tracked == 1:
	# 	global x, y, w, h
	# 	totalDepth = 0
	# 	numOfPixel = 0
	# 	for i in range (x, x+w):
	# 		for j in range (y, y+h):
	# 			if (depthFrame[i, j] > 600):
	# 				totalDepth = totalDepth + depthFrame[i,j]
	# 				numOfPixel = numOfPixel + 1
	# 	msg.distance_err = totalDepth/numOfPixel

def main():
	global pub
	rospy.init_node('image_detection_node', anonymous=True)
	pub = rospy.Publisher('follower_err', follower_err, queue_size=10)
	rospy.Subscriber("/camera/rgb/image_raw", Image, imageCallback)
	rospy.Subscriber("camera/depth/image_raw", Image, depthCallback)

	if not rospy.is_shutdown():
		try:
			rospy.spin()
		except KeyboardInterrupt:
			msg.is_obj_being_tracked = 0
			msg.direction_err = 0.0
			msg.distance_err = 1000.0
			pub.publish(msg)
			print("Shutting down")
			cv2.destroyAllWindows()

if __name__ == '__main__':
	main()