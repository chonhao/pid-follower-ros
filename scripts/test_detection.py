#!/usr/bin/env python

import rospy
import cv2
import argparse
import time
from pid_follower.msg import follower_err
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

msg = follower_err()
tracker = cv2.TrackerMedianFlow_create()
initBB = None
global x, y, w, h
global depthImage
depthImage = None
# x = 0.0
# y = 0.0
# w = 0.0
# h = 0.0
# objectDepth = 0.0

def imageCallback(data):
	global tracker 
	global initBB

	#change ros image to opencv image
	try:
		bridge = CvBridge()
		frame = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)
    
	#resize opencv image and change to grayscale
	(W, H) = (500, 375)
	frame = cv2.resize(frame, (W,H), interpolation=cv2.INTER_CUBIC)
	# frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	#Getting depth map

	#apply tracker
	if initBB is not None:
		# grab the new bounding box coordinates of the object
		(success, box) = tracker.update(frame)

		# check to see if the tracking was a success
		if success:
			msg.is_obj_being_tracked = 1.0
			(x, y, w, h) = [int(v) for v in box]
			cv2.rectangle(frame, (x, y), (x + w, y + h),
				(0, 255, 0), 2)

			# Calculating distance from center IN PIXEL
			FrameCenter = W/2
			BoxCenter = (int)(x+w/2)
			PixelFromCenter = BoxCenter - FrameCenter
			msg.direction_err = PixelFromCenter

			#calculate depth of object
			if depthImage is not None:
				BoxCenterX = (int)(x+w/2)
				BoxCenterY = (int)(y+h/2)
				objectDepth = depthImage [BoxCenterX, BoxCenterY]
				print(objectDepth)
				msg.distance_err = objectDepth

		else:
			initBB = None
			msg.is_obj_being_tracked = 0
			msg.direction_err = 0.0
			msg.distance_err = 0.0

		

		# initialize the set of information we'll be displaying on
		# the frame
		info = [
			# ("Tracker", args["tracker"]),
			("Success", "Yes" if success else "No"),
			# ("FPS", "{:.2f}".format(fps.fps())),
			("Distance from center(px)",msg.direction_err)
			# ("Depth(mm)",msg.distance_err)
		]

		# loop over the info tuples and draw them on our frame
		for (i, (k, v)) in enumerate(info):
			text = "{}: {}".format(k, v)
			cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
				cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
	
	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	pub.publish(msg)

	if key == ord("s"):
		initBB = cv2.selectROI("Frame", frame, fromCenter=False,
			showCrosshair=True)
		tracker.init(frame, initBB)

	#calculate the distance and return

	#show windows for debugging
	# cv2.imshow("Image window", frame)
	# cv2.waitKey(3)

def depthCallback(data):
	try:
		bridge = CvBridge()
		depthImage = bridge.imgmsg_to_cv2(data,"passthrough")
		(W, H) = (500, 375)
		depthImage = cv2.resize(depthImage, (W,H), interpolation=cv2.INTER_CUBIC)
		# cv2.imshow("Depth", depthImage)
		# key = cv2.waitKey(1) & 0xFF
	except CvBridgeError as e:
		print(e)

def main():
	#Initiate tracker
	global pub
	rospy.init_node('image_detection_node', anonymous=True)
	pub = rospy.Publisher('follower_err', follower_err, queue_size=10)
	rospy.Subscriber("/camera/rgb/image_raw", Image, imageCallback)
	rospy.Subscriber("camera/depth/image_raw", Image, depthCallback)

	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		msg.is_obj_being_tracked = 0
		msg.direction_err = 0.0
		msg.distance_err = 0.0
		pub.publish(msg)
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()