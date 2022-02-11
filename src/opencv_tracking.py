#!/usr/bin/env python

## @package opencv_tracking
#
# it make uses of openCV libraries to track the ball moving within the map so that
# the robot can follow it

import sys
import time

import numpy as np
from scipy.ndimage import filters

import imutils # image processing functions easier with OpenCV and both Python 2.7 and Python 3. 

# OpenCV 
import cv2

# Ros libraries
import roslib
import rospy

# import ROS Messages
from sensor_msgs.msg import CompressedImage, LaserScan 
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float64
from nav_msgs.msg import Odometry
from random import randint
import math 

VERBOSE = False

#publisherHeadPos = None

## define colours limits
# in our simulation we have different colourd balls: green, black, red, yellow, blue, magenta
greenLower = (50, 50, 20)
greenUpper = (70, 255, 255)

blackLower = (0, 0, 0)
blackUpper = (5, 50, 50)

redLower = (0, 50, 50)
redUpper = (5, 255, 255)

yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)

blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)

magentaLower = (125, 50, 50)
magentaUpper = (150, 255, 255)


## class track_ball
#
# used to track the ball moving in the environment
# it makes use of cv2
class track_ball:
	## method __init__
	# 
	# initialization of the class
	def __init__(self):
		# we need the following
		#	- ball dimensions described by its center and radius
		#	- boolean to check if ball if visible or not
		#	- boolean to check if the ball is moving or not
		# 	- boolean to check if the robot is close to the ball or not
		#	- robot behavior
		self.center = None
        	self.radius = None
		self.ball_visible = False
       		self.ball_stop = False
        	self.near_ball = False
        	self.behaviour = None
		self.at_ball = False
		self.colour = None
		self.current_position = None
		self.room = None

		self.regions_ = {
			'right': 0,
			'fright': 0,
			'front': 0,
			'fleft': 0,
			'left': 0,
		}
		
		# publish robot velocity
		self.vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
	    	# publish if ball detected
		self.publisherBall = rospy.Publisher("/ball_visible", Bool, queue_size=1)
		# publish if the robot is close to the ball
		self.publisherBall = rospy.Publisher("/at_room", Bool, queue_size=1)
		# subscribe to camera
		self.cam_sub = rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.callback,  queue_size=1)
		# subscribe odometry of the robot
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.get_odom_data)
		# subscribe laser of the robot
		self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.get_laser_data)
		# subscribe to get which room it should move to 
		self.room_sub = rospy.Subscriber('/room_command', LaserScan, self.get_rooom_data)
		# subscriber to current behaviour
        	rospy.Subscriber("/behavior", String, self.get_behavior)

	## method get_behavior
	#
	# subscriber callback to the behavior topi
	def get_behavior(self,state):
		self.behavior = state.data

	## method get_room_data
	#
	# subscriber callback to get current room
	def get_room_data(self,room):
		self.room = room.data	

	## method get_odom_data
	#
	# subscriber callback to get odometry of the robot
	def get_odom_data(self,odom):
		self.current_position = odom.data

	## method get_laser_data
	#
	# subscriber callback to get laser data
	def get_laser_data(self,laser):
		self.regions = {
			'right':  min(min(msg.ranges[0:143]), 10),
            		'fright': min(min(msg.ranges[144:287]), 10),
            		'front':  min(min(msg.ranges[288:431]), 10),
            		'fleft':  min(min(msg.ranges[432:575]), 10),
            		'left':   min(min(msg.ranges[576:713]), 10),	
		}
			

	## method follow_ball_obstacles
	#
	# method to set the robot velocity so that it can follow the ball without hitting obstacles
	def follow_ball_obstacles(self):
		regions = self.regions_
		msg = Twist()
		angular_z = -0.003*(self.center[0] - 400)
        	linear_x = -0.01*(self.radius - 100)
        	state_description = ''

		# linear velocity saturation
		if linear_x > 0.4:
			linear_x = 0.4		
		
		d0 = 0.1
        	d = 0.15

		if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
            		state_description = 'case 1 - nothing'
			# # go towards the ball
			twist_msg = Twist()
			twist_msg.linear.x = linear_x
			twist_msg.angular.z = angular_z
			self.vel_pub.publish(twist_msg)
		elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
            		state_description = 'case 2 - front'
            		# turn
            		twist_msg = Twist()
            		twist_msg.linear.x = 0
            		twist_msg.angular.z = 0.3
            		self.vel_pub.publish(twist_msg)
		elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
            		state_description = 'case 3 - fright'
            		# turn left a little
			twist_msg = Twist()
			twist_msg.linear.x = 0
			twist_msg.angular.z = 0.3
			self.vel_pub.publish(twist_msg)

        	elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
            		state_description = 'case 4 - fleft'
            		# turn right a little
            		twist_msg = Twist()
            		twist_msg.linear.x = 0
            		twist_msg.angular.z = -0.3
            		self.vel_pub.publish(twist_msg)
            
        	elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
            		state_description = 'case 5 - front and fright'
            		# turn right a little
            		twist_msg = Twist()
            		twist_msg.linear.x = 0
            		twist_msg.angular.z = 0.3
            		self.vel_pub.publish(twist_msg)

        	elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
            		state_description = 'case 6 - front and fleft'
            		# turn right a little
            		twist_msg = Twist()
            		twist_msg.linear.x = 0
            		twist_msg.angular.z = -0.3
            		self.vel_pub.publish(twist_msg)

        	elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
            		state_description = 'case 7 - front and fleft and fright'
            		# go towards the ball
            		twist_msg = Twist()
            		twist_msg.angular.z = linear_x
            		twist_msg.linear.x = angular_z
            		self.vel_pub.publish(twist_msg)

        	elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
            		state_description = 'case 8 - fleft and fright'
            		# go towards the ball
            		twist_msg = Twist()
            		twist_msg.angular.z = linear_x
            		twist_msg.linear.x = angular_z
            		self.vel_pub.publish(twist_msg)
        	else:
           		state_description = 'unknown case'
            
	## method mask_colour
	#
	#  method to decide which colour
	def mask_colour(self, maskGreen, maskBlack, maskRed, maskYellow, maskBlue, maskMagenta):
		sumGreen = np.sum(maskGreen)
		sumBlack = np.sum(maskBlack)
		sumRed = np.sum(maskRed)
		sumBlue = np.sum(maskBlue)
		sumMagent = np.sum(maskMagent)

		sumArray = np.array([sumGreen, sumBlack, sumRed, sumBlue, sumMagent])
		max_ind = np.argmax(sumArray)
	
		# return mask 
		if max_ind == 0:
			return [maskGreen, 'Green']
		elif max_ind == 1:
			return [maskBlack, 'Black']
		elif max_ind == 2:
			return [maskRed, 'Red']
		elif max_ind == 3:
			return [maskYellow, 'Yellow']
		elif max_ind == 4:
			return [maskBlue, 'Blue']
		elif max_ind == 5:
			return [maskMagenta, 'Magenta']
		else:
			return [maskGreen, 'None']
		
	## method follow_ball
	#
	# follow the ball
	def follow_ball(self):
		if self.near_ball:
			# if near enough to the ball start following it
			self.follow_ball_obstacles()
		else:
			# if not near enough, it should get closer
			twist_msg = Twist()
			twist_msg.linear.x = 0.4
			self.vel_pub.publish(twist_msg)


		
	##  method callback
	#
	# callback function of topic subscribed
	# we use it to read information on the detection and the converted image
	def callback(self, ros_data):
		if not self.ball_stop:
			if VERBOSE:
				print('Image received. Type: "%s"' % ros_data.format)
			angular_z = None
			linear_x = None
			## cv2 conversion
			np_arr = np.fromstring(ros_data.data, np.uint8)
			# the function imdecode compresses the image and stores it in memory buffer         			
			image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			
			# thresholds
			greenLower = (50, 50, 20)
            		greenUpper = (70, 255, 255)
			# Blurs image using a Gaussian filter.
			blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
			# convert image from one color space to another.
            		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
			# threshold operations    			        
			maskGreen  = cv2.inRange(hsv, greenLower, greenUpper)
			maskBlack  = cv2.inRange(hsv, blackLower, blackUpper)
			maskGreen  = cv2.inRange(hsv, redLower, redUpper)
			maskGreen  = cv2.inRange(hsv, yellowLower, yellowUpper)
			maskGreen  = cv2.inRange(hsv, blueLower, blueUpper)
			maskGreen  = cv2.inRange(hsv, magentaLower, magentaUpper)			
			
			# perform erosion on the image
            		mask = cv2.erode(mask_colour[0], None, iterations=2)
			# do opposite
			mask = cv2.dilate(mask, None, iterations=2)

			# Contours -> curve joining all the continuous points 
			# having same color or intensity. 
			# The contours are a useful tool for shape analysis and 
			# object detection and recognition.
			# For better accuracy apply threshold
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
				                    cv2.CHAIN_APPROX_SIMPLE)
			cnts = imutils.grab_contours(cnts)
			self.center = None

			# only proceed if at least one contour was found
			if len(cnts) > 0:
			    	# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and
				# centroid
				c = max(cnts, key=cv2.contourArea)
				# get circle which completely covers the object with minimum area.
				((x, y), radius) = cv2.minEnclosingCircle(c)
				self.radius = radius
				# M dictionary of all moments values calculated 
				M = cv2.moments(c)
				self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				# ball is visible
				self.ball_visible = True

				# only proceed if the radius meets a minimum size
				if radius > 10:
					# draw the circle and centroid on the frame using circle()
					# then update the list of tracked points
					cv2.circle(image_np, (int(x), int(y)), int(radius),
				    		(0, 255, 255), 2)
					cv2.circle(image_np, self.center, 5, (0, 0, 255), -1)
					# robot is near the ball
					self.near_ball = True
				else:
					self.near_ball = False

			else:
				self.ball_visible = False
			
			# we need to publish if the ball is visible or not
            		self.publisherBall.publish(self.ball_visible)

			# update the points queue
			cv2.imshow('window', image_np)
			cv2.waitKey(2)

		    	# if behaviour is play, follow the ball
		    	if self.behaviour == "normal_track" or self.behaviour == "find_track":
		    		if self.ball_detected and self.near_ball:
		    	        	angular_z = -0.003*(self.center[0] - 400)
		    	        	linear_x = -0.01*(self.radius - 100)
					# if the robot almost still
					if abs(angular_z) < 0.02 and abs(linear_x) < 0.02 :
		    	            		self.at_ball = True
						self.colour = mask_colour[1]
					# save ball info	
					self.save_info(self.colour)
				
					rospy.sleep(2)
					
					if self.behaviour == "find_track":
						room_colour = rospy.get_param(self.room)
						if self.colour == room_colour:
		                    			rospy.loginfo("NODE OPENCV TRACKING: The correct room %s (%s) has been found. Switch to play behaviour...", self.room, room_colour)
                            				self.pub_room_found.publish(True)
                        			else:
                            				rospy.loginfo("NODE OPENCV TRACKING: The correct room %s (%s) has not been found. Switch to find behaviour", self.room, room_colour)
                            				self.pub_room_found.publish(False)			
		    	   			# publish info
						self.pub_reach.publish(self.at_ball)
						self.at_ball = False
					else:
						# follow ball
						self.follow_ball()
				# if ball visible
				if mask_colour[1] != self.colour:
					self.pub_ball.publish(self.ball_visible)
				else:
					self.pub_ball.publish(False)		
			
	## method store_ball
	#
	# store position of the detected ball
	def store_ball(self,colour):
		rospy.loginfo("NODE OPENCV TRACKING: save %s ball location", colour)
		ball_position = [self.current_position.x, self._current_position.y]
		rospy.loginfo("NODE OPENCV TRACKING: ball position is %s", srt(ball_pos))
	
		rospy.set_param(colour, ball_position)	
	

## function main
#
# 
def main(args):
	# init trackin node
	rospy.init_node('opencv_tracking', anonymous = True)
	
	global publisherHeadPos

	rate = rospy.Rate(100)
	
	publisherHeadPos = rospy.Publisher("/robot/joint_position_controller/command", Float64, queue_size=1)

	# init class




















+
	trackBall = track_ball()
	
	while not rospy.is_shutdown():
		# when the ball stops, the robot stops and moves its head
		if trackBall.ball_stop:
			# move robot head
			move_head()
			rospy.sleep(2)
			trackBall.ball_stop = False
			rospy.loginfo("NODE OPCV_TRACK: tracking ball")
		rate.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print ("Shutting down ROS Image feature detector module")
    	cv2.destroyAllWindows()

if __name__ == '__main__':	
	main(sys.argv)

