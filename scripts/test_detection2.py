#!/usr/bin/env python

import rospy
import cv2, sys, os
import argparse
import time
from pid_follower.msg import follower_err
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

msg = follower_err()

tracker = None
initBB = None

ADJUST_PID = True

BLURRINESS_THRESHOLD = 750

def get_blurriness(frame):
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	fm = cv2.Laplacian(gray, cv2.CV_64F).var()
	return fm

def is_image_blurry(frame):
	fm = get_blurriness(frame)
	if fm < BLURRINESS_THRESHOLD:
		print("blurry")
		return True
	return False

def imageCallback(data):
	# get image from ros
	try:
		bridge = CvBridge()
		imageFrame = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	# resize
	(W, H) = (500, 375)
	alpha = 2.2
	beta = 50.0
	imageFrame = cv2.resize(imageFrame, (W,H), interpolation=cv2.INTER_CUBIC)
	imageFrame = cv.convertScaleAbs(imageFrame, alpha=alpha, beta=beta)

	# skip frame if blurry
	if is_image_blurry(imageFrame):
		return

	# DEBUG: add bounding box on start
	if ADJUST_PID:
		initBB = (W-50, 375/2-50, 50, 100)
		tracker = cv2.TrackerMedianFlow_create()
		tracker.init(imageFrame, initBB)
		global ADJUST_PID
		ADJUST_PID = False

	# tracker
	msg.is_obj_being_tracked = 0
	global initBB
	if initBB is not None:
		# update box from tracker
		global tracker
		(success, box) = tracker.update(imageFrame)

		if success:
			global x, y, w, h
			msg.is_obj_being_tracked = 1
			(x, y, w, h) = [int(v) for v in box]
			cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)

			# calc distance from center IN PIXEL
			FrameCenter = W/2
			BoxCenter = (int)(x+w/2)
			PixelFromCenter = BoxCenter - FrameCenter
			# msg.direction_err = PixelFromCenter
			degreePerPixel = 60/500
			

		else: # lost track
			initBB = None
			msg.is_obj_being_tracked = 0
			msg.direction_err = 0.0
			print "Blurriness: ",
			print get_blurriness(imageFrame)

		# show info on screen
		info = [
			("Success", "Yes" if success else "No"),
			("Direction from center ",msg.direction_err),
			("Distance ERR ", msg.distance_err)
		]
		for (i, (k, v)) in enumerate(info):
			text = "{}: {}".format(k, v)
			cv2.putText(imageFrame, text, (10, H - ((i * 20) + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

	# show frame	
	cv2.imshow("Image Detection Node", imageFrame)
	key = cv2.waitKey(1) & 0xFF
	pub.publish(msg)

	#select tracking object
	if key == ord("s"):
		initBB = cv2.selectROI("Image Detection Node", imageFrame, fromCenter=False, showCrosshair=True)
		tracker = None
		tracker = cv2.TrackerMedianFlow_create()
		tracker.init(imageFrame, initBB)
	
	if key == ord("d"):
		ADJUST_PID = True

def depthCallback(data):
	# get depth from ros
	try:
		bridge = CvBridge()
		depthFrame = bridge.imgmsg_to_cv2(data, "passthrough")

		#resize
		(W, H) = (500, 375)
		depthFrame = cv2.resize(depthFrame, (W,H), interpolation=cv2.INTER_CUBIC)
	except CvBridgeError as e:
		print e

	if msg.is_obj_being_tracked == 1:
		global x, y, w, h
		boxCenter_x = (x+(w/2))
		boxCenter_y = (y+(h/2))

		depthList = []
		threshold_x = w/4
		threshold_y = h/4
		for i in range(-1, 2):
			for j in range(-1, 2):
				depth = depthFrame[boxCenter_y + j * threshold_y, boxCenter_x + i * threshold_x]
				if depth>600:
					depthList.append(depth)
		centreDepth = depthFrame[boxCenter_y, boxCenter_x] > 600
		if centreDepth > 600:
			depthList.append(centreDepth)
		depthList.sort()

		finalDepth = depthList[(len(depthList)+1)/2]
		print depthList
		msg.distance_err = finalDepth

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