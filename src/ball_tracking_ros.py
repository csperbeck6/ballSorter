#!/usr/bin/env python

# import the necessary packages
from collections import deque
import numpy as np
import imutils
import cv2
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

#make our publisher a global variable so it's visible in the callback function
pub = rospy.Publisher('ballDetectLocationPub', Point, queue_size=100)

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
HSVLower = (0, 175, 108) #red = (0, 175, 64) # blue (92, 183, 0) # purple calibration dots (113, 24, 19)
HSVUpper = (206, 255, 255) #red = (200, 255, 255) # blue (125, 255,65) # purple calibration dots (166, 255, 255)
pts = deque(maxlen=64)
color = 0 # red = 0, blue = 1
 
camera = cv2.VideoCapture(0)
print "camera opened"

w = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
h = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
print "w :"
print w
print "h :"
print h

def ballRemovedSub(msg):

	global ballRemoved
	ballRemoved = msg.data	


#run our procedure on startup
if __name__ == '__main__':
	#initialize our node
	rospy.init_node('ball_tracking_ros', anonymous=True)

	#subscriber
	rospy.Subscriber("ballRemovedPub", Bool, ballRemovedSub)

        rate = rospy.Rate(50) # 5hz
	
	# keep looping
        while not rospy.is_shutdown():

		# grab the current frame
		(grabbed, frame) = camera.read()
	 	
		# resize the frame, blur it, and convert it to the HSV
		# color space
		frame = imutils.resize(frame, width=600)
		# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	 
		# construct a mask for the color "red", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, HSVLower, HSVUpper) # initially red
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		print "len of cnts red"
		print len(cnts)
		print "color"
		print color
		#finished picking up the red balls, time to pick up blue ones
		if(len(cnts) == 0 and color == 0):
			print "Time to detect blue balls"
			# set new ball color to look for to be blue balls
			HSVLower = (95, 170, 22)
			HSVUpper = (125, 255,126)
			color = 1

			# grab the current frame
			(grabbed, frame) = camera.read()
		 	
			# resize the frame, blur it, and convert it to the HSV
			# color space
			frame = imutils.resize(frame, width=600)
			# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		 
			# construct a mask for the color "red", then perform
			# a series of dilations and erosions to remove any small
			# blobs left in the mask
			mask = cv2.inRange(hsv, HSVLower, HSVUpper) # initially red
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)

			# find contours in the mask and initialize the current
			# (x, y) center of the ball
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

		#finished picking up the blue balls, time to shut down
		elif(len(cnts) == 0 and color == 1):
			# cleanup the camera and close any open windows
			camera.release()
			cv2.destroyAllWindows()

		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
	
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame
			cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

			#publish the found center of the ball
			point   = Point()
			point.x = center[0]
			point.y = center[1]
			# send the color of the ball to the arm so it knows what direction and bin to place the ball into
			# red = 0, blue = 1
			if(color == 0):
				point.z = 0
			elif(color == 1):
				point.z = 1

			pub.publish(point)

			rospy.sleep(1)
 
			# update the points queue
			pts.appendleft(center)

			print "ballRemoved"
			print ballRemoved

			while(ballRemoved == False):

				# grab the current frame
				(grabbed, frame) = camera.read()

				# resize the frame, blur it, and convert it to the HSV color space
				frame = imutils.resize(frame, width=600)

				# draw the circle and centroid on the frame
				cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)

				# show the frame to our screen
				cv2.imshow("Frame", frame)
				key = cv2.waitKey(1) & 0xFF

	# cleanup the camera and close any open windows
	camera.release()
	cv2.destroyAllWindows()
