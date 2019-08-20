#!/usr/bin/env python

import rospy
import cv2, sys, os
import argparse
import time
from pid_follower.msg import follower_err
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

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

ADJUST_PID = False

blurriness_threshold = 600

def get_blurriness(frame):
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	fm = cv2.Laplacian(gray, cv2.CV_64F).var()
	return fm

def is_image_blurry(frame):
	return False
	fm = get_blurriness(frame)
	if fm < blurriness_threshold:
		print("blurry")
		return True
	return False

def imageCallback(data):
	#change ros image to opencv image
	try:
		bridge = CvBridge()
		imageFrame = bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	#resize opencv image
	(W, H) = (500, 375)
	# alpha = 2.7					#alpha contrast, beta brightness
	# beta = 10.0
	imageFrame = cv2.resize(imageFrame, (W,H), interpolation=cv2.INTER_CUBIC)
	# imageFrame = cv2.convertScaleAbs(imageFrame, alpha=alpha, beta=beta)

	imageFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
	

	# define range of blue color in HSV
	lower_blue = np.array([200 /2.0,50  /100.0*255,30  /100.0*255])
	upper_blue = np.array([270 /2.0,100 /100.0*255,100 /100*255])

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(imageFrame, lower_blue, upper_blue)
	# Bitwise-AND mask and original image
	imageFrame = cv2.bitwise_and(imageFrame,imageFrame, mask= mask)

	if is_image_blurry(imageFrame):
		return

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
			print "Blurriness: ",
			print get_blurriness(imageFrame)

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
	
	if key == ord("d"):
		ADJUST_PID = True

def depthCallback(data):
	try:
		bridge = CvBridge()
		depthFrame = bridge.imgmsg_to_cv2(data, "passthrough")
		(W, H) = (500, 375)
		depthFrame = cv2.resize(depthFrame, (W,H), interpolation=cv2.INTER_CUBIC)
		# depthf = np.uint8(depthFrame)

		# # smallFrame = cv2.resize(depthf, (W,H), 0.2, 0.2)
		# smallFrame = depthf
		# temp = cv2.cvtColor(depthFrame, cv2.CV_8U)
		# temp = cv2.UMat(temp)
		# temp = cv2.inpaint(temp, (temp == 0), 5.0, flags=cv2.INPAINT_TELEA)
		# smallFrame = cv2.resize(smallFrame, (W,H))
		# smallFrame.copyTo(depthFrame, (depthFrame == 0))

		cv2.imshow("DepthFrame", depthFrame)
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

	#get box center's depth data (with filtering)
	# if msg.is_obj_being_tracked == 1:
	# 	global x, y, w, h
	# 	BoxCenterX = (x+(w/2))
	# 	BoxCenterY = (y+(h/2))
	# 	error = False
	# 	pta = depthFrame[BoxCenterY - 2, BoxCenterX - 2]
	# 	diff1 = depthFrame[BoxCenterY, BoxCenterX] - pta
	# 	if diff1 < -200 or diff1 > 200:
	# 		error = True
	# 	ptb = depthFrame[BoxCenterY - 2, BoxCenterX]
	# 	diff2 = depthFrame[BoxCenterY, BoxCenterX] - ptb
	# 	if diff2 < -200 or diff2 > 200:
	# 		error = True
	# 	ptc = depthFrame[BoxCenterY - 2, BoxCenterX + 2]
	# 	diff3 = depthFrame[BoxCenterY, BoxCenterX]
	# 	if diff3 < -200 or diff3 > 200:
	# 		error = True
	# 	ptd = depthFrame[BoxCenterY, BoxCenterX - 2]
	# 	diff4 = depthFrame[BoxCenterY, BoxCenterX] - ptd
	# 	if diff4 < -200 or diff4 > 200:
	# 		error = True
	# 	pte = depthFrame[BoxCenterY, BoxCenterX + 2]
	# 	diff5 = depthFrame[BoxCenterY, BoxCenterX] - pte
	# 	if diff5 < -200 or diff5 > 200:
	# 		error = True
	# 	ptf = depthFrame[BoxCenterY + 2, BoxCenterX - 2]
	# 	diff6 = depthFrame[BoxCenterY, BoxCenterX] - ptf
	# 	if diff6 < -200 or diff6 > 200:
	# 		error = True
	# 	ptg = depthFrame[BoxCenterY + 2, BoxCenterX]
	# 	diff7 = depthFrame[BoxCenterY, BoxCenterX] - ptg
	# 	if diff7 < -200 or diff7 > 200:
	# 		error = True
	# 	pth = depthFrame[BoxCenterY + 2, BoxCenterX + 2]
	# 	diff8 = depthFrame[BoxCenterY, BoxCenterX] - pth
	# 	if diff8 < -200 or diff8 > 200:
	# 		error = True
	# 	if error == False:
	# 		if depthFrame[BoxCenterY, BoxCenterX] > 600:
	# 			msg.distance_err = (depthFrame[BoxCenterY, BoxCenterX]/10)*10
	# 			print("Success lah")
	# 		else:
	# 			msg.distance_err = -1
	# 	else:
	# 		totalDepth = 0
	# 		totalNum = 8
	# 		if (pta > 600):
	# 			totalDepth = totalDepth + pta
	# 		else:
	# 			totalNum = totalNum - 1
	# 		if (ptb > 600):
	# 			totalDepth = totalDepth + ptb
	# 		else:
	# 			totalNum = totalNum - 1
	# 		if (ptc > 600):
	# 			totalDepth = totalDepth + ptc
	# 		else:
	# 			totalNum = totalNum - 1
	# 		if (ptd > 600):
	# 			totalDepth = totalDepth + ptd
	# 		else:
	# 			totalNum = totalNum - 1
	# 		if (pte > 600):
	# 			totalDepth = totalDepth + pte
	# 		else:
	# 			totalNum = totalNum - 1
	# 		if (ptf > 600):
	# 			totalDepth = totalDepth + ptf
	# 		else:
	# 			totalNum = totalNum - 1
	# 		if (ptg > 600):
	# 			totalDepth = totalDepth + ptg
	# 		else:
	# 			totalNum = totalNum - 1
	# 		if (pth > 600):
	# 			totalDepth = totalDepth + pth
	# 		else:
	# 			totalNum = totalNum - 1
	# 		msg.distance_err = ((totalDepth / totalNum)/10)*10
	# 		print("You have error ah")
	
	#get 9 dots of depth data in box
	# if msg.is_obj_being_tracked == 1:
	# 	global x, y, w, h
	# 	BoxCenterX = (x+(w/2))
	# 	BoxCenterY = (y+(h/2))
	# 	totalDepth = depthFrame[BoxCenterY - 10, BoxCenterX - 10] \
	# 		+ depthFrame[BoxCenterY - 10, BoxCenterX] \
	# 		+ depthFrame[BoxCenterY - 10, BoxCenterX + 10] \
	# 		+ depthFrame[BoxCenterY, BoxCenterX - 10] \
	# 		+ depthFrame[BoxCenterY, BoxCenterX] \
	# 		+ depthFrame[BoxCenterY, BoxCenterX + 10] \
	# 		+ depthFrame[BoxCenterY + 10, BoxCenterX - 10] \
	# 		+ depthFrame[BoxCenterY + 10, BoxCenterX] \
	# 		+ depthFrame[BoxCenterY + 10, BoxCenterX + 10]
	# 	objectDepth = totalDepth / 9
	# 	if objectDepth > 600:
	# 		msg.distance_err = objectDepth
	# 	else:
	# 		msg.distance_err = -1

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
	rospy.Subscriber("camera/depth/image_smoothed", Image, depthCallback)

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
