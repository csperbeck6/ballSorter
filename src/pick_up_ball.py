#!/usr/bin/env python

#import necessary libraries
import pypot.dynamixel
import itertools
import rospy
import math
from geometry_msgs.msg import Point, Point32
from time import sleep
from geometry_msgs.msg import Polygon
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

#make our publisher a global variable so it's visible in the callback function
pub = rospy.Publisher('ballRemovedPub', Bool, queue_size=100)

global heightServo2
heightServo2 =  12.57  # 13.97  cm = 5.5 inches
global lenServoBtwn23
lenServoBtwn23 = 10.73  # 10.16  cm = 4 inches between servos 2 and 3
global lenServoBtwn34 
lenServoBtwn34= 19.7  # 22.5425 cm = 8.875 inches between servos 3 and tip of claw


#get all available ports your robot is connected to, assume the first one
ports = pypot.dynamixel.get_available_ports()
print('Available ports:', ports)
if not ports:
    raise IOError('No port available.')
port = ports[0]
print('Using the first on the list' + port)

#connect to the port
dxl_io = pypot.dynamixel.DxlIO(port)
print('Connected...')

#find servo_ids
found_ids = dxl_io.scan([1,2, 3,4, 5])
print('Found ids:', found_ids)

#there should be five servos, exit upon failure
if len(found_ids) < 5:
    raise IOError('You should connect at least five motors on the bus for this test.')

#start procedure
print('Ready!')

#
# Global constants that linear speed
# SPEED = 25
#
SPEED = 25    

#
# Enable servos and set the speed
#
dxl_io.enable_torque(found_ids)
speed = dict(zip(found_ids, itertools.repeat(SPEED)))
dxl_io.set_moving_speed(speed)

def lawOfCosines(sideA, sideB, sideC):
    #calulcate angle A
    cosA = (sideB ** 2 + sideC ** 2 - sideA ** 2) / (2 * sideB * sideC)
    angleRadiansA = math.acos(cosA)
    angleDegreesA = (angleRadiansA *180) / math.pi
    #print "angleDegreesA: "
    #print angleDegreesA

    #calulcate angle B
    cosB = (sideA ** 2 + sideC ** 2 - sideB ** 2) / (2 * sideA * sideC)
    angleRadiansB = math.acos(cosB)
    angleDegreesB = (angleRadiansB *180) / math.pi
    #print "angleDegreesB: "
    #print angleDegreesB

    #calulcate angle C
    angleDegreesC = 180 - (angleDegreesA + angleDegreesB);
    #print "angleDegreesC: "
    #print angleDegreesC

    return [angleDegreesA, angleDegreesB, angleDegreesC]

def closePincher():
    pincherPos = {5: 80}
    dxl_io.set_goal_position(pincherPos)

    # Allow time for the servo to get into place
    rospy.sleep(3) # Sleeps for 3 sec

    return 

def openPincher():
    pincherPos = {5: 10}
    dxl_io.set_goal_position(pincherPos)

    # Allow time for the servo to get into place
    rospy.sleep(3) # Sleeps for 3 sec

    return 

def pickUpBall(ballLocXCentimeter,  ballLocYCentimeter):
    hypotenuse = math.sqrt(math.sqrt((ballLocYCentimeter ** 2 + ballLocXCentimeter ** 2))** 2 + heightServo2 ** 2)
    print "pickUpBall hypotenuse"
    print hypotenuse
    [alpha, beta, gamma] = lawOfCosines(lenServoBtwn34, lenServoBtwn23, hypotenuse)

    angleRadians = math.acos(abs(heightServo2/hypotenuse)) # CAH
    alphaDegrees = (angleRadians *180) / math.pi

    turnServo1(ballLocXCentimeter, ballLocYCentimeter)

    servoPos = {4: 10}
    dxl_io.set_goal_position(servoPos)

    servoPos = {3: 180 - gamma}
    dxl_io.set_goal_position(servoPos)

    servoPos = {2: 180 - (alphaDegrees + alpha)}
    dxl_io.set_goal_position(servoPos)
    print "pickUpBall alpha"
    print 180 - (alphaDegrees + alpha)
    print "pickUpBall beta"
    print beta
    print "pickUpBall gamma"
    print 180 - gamma

    rospy.sleep(6) # Sleeps for 6 sec

    closePincher()

    return 

def turnServo1(x, y):
    if x < 0:
	direction = -1
    else:
	direction = 1

    #hypo = math.sqrt(x ** 2 + y ** 2)
    #print hypo
    #[alpha, beta, gamma] = lawOfCosines(x, hypo, y);
    #print "gamma"
    #print gamma

    angleRadians = math.atan(abs(x/y)) # TOA
    alphaDegrees = (angleRadians *180) / math.pi

    servoPos = {1: direction * alphaDegrees}
    dxl_io.set_goal_position(servoPos)

    rospy.sleep(2) # Sleeps for 2 sec

    return 

def dropIntoBasket(leftOrRight):

    # to turn right, leftOrRight = -1
    # to turn left, leftOrRight = 1

    #Make right angle with the arm
    pos = {2: 0}
    dxl_io.set_goal_position(pos)
    pos = {3: 90}
    dxl_io.set_goal_position(pos)
    pos = {4: 0}
    dxl_io.set_goal_position(pos)

    # now rotate left
    pos = {1: leftOrRight * 90}
    dxl_io.set_goal_position(pos)

    # Allow time for the servo to get into place
    rospy.sleep(5) # Sleeps for 5 sec

    # drop ball by opening pincher
    openPincher()     

    return 

def goToHomePos():
    newloc = dict(zip(found_ids, itertools.repeat(0)))
    dxl_io.set_goal_position(newloc)

    # Allow time for the servo to get into place
    rospy.sleep(5) # Sleeps for 5 sec
    return 

def listener():

	#initialize our node
	rospy.init_node('ballDetectLocationSub', anonymous=True)

	pub.publish(True);
	print "PUBLSIHED TRUE"

	#subscriber
	rospy.Subscriber("ballDetectLocationPub", Point, ballDetectLocationSub)


def ballDetectLocationSub(data):

	#define centers of ball locations
	global x
	global y
	global color

	# box dimensions are 20 x 15 inches = 50.8 x 38.1 cm
	# the first servo has an ofset of 4 inches (10.16 cm) from the base of the camera 
	servoArmOffsetY = 115 # 10.16/38.1 * 450
	servoArmOffsetX = 300 # 600/2
	# w = camera.get(cv2.CAP_PROP_FRAME_WIDTH) = 600
	# h = camera.get(cv2.CAP_PROP_FRAME_HEIGHT) = 450

	# calculate x coordinate from arm by converting from the camera corrdinate system to the dyna arms
	if data.x > servoArmOffsetX: # positive x axis of the arm
		 x = (data.x - servoArmOffsetX)/600 * 50.8
	else: # negative x axis of the arm
		 x = -(servoArmOffsetX - data.x)/600 * 50.8

	# calculate y coordinate from arm by converting from the camera corrdinate system to the dyna arms
	y = (data.y - servoArmOffsetY)/450 * 38.1

	#set the color detected from the camera
	color = data.z

	pub.publish(False);
	print "data.x "
	print data.x

	print "data.y"
	print data.y

	print "color"
	print data.z
 
#run our procedure on startup
if __name__ == '__main__':
        listener()

	rospy.sleep(5)
  
	(x1, y1) = (11.8533333333, 21.59) # 3.81 cm = 1.5 inches and 7.62 cm =  3 inches
	rate = rospy.Rate(50) # 5hz

	# keep looping
        while not rospy.is_shutdown():
		#return to home position
		goToHomePos() 

		#calculate new ball location and move to that position
		print "x: main "
		print x

		print "y: main"
		print y

		pickUpBall(x, y)
	    	#servoPos = {3: 108.198588041}
	    	#dxl_io.set_goal_position(servoPos)

	    	#servoPos = {2: 57.6969141249}
	    	#dxl_io.set_goal_position(servoPos)

		#drop the ball in the appropriate basket
		if(color == 0):
			dropIntoBasket(1)
		elif(color == 1):
			dropIntoBasket(-1)

		pub.publish(True);



	
	


