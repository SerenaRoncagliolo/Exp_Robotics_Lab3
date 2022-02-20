#!/usr/bin/env python

## @package behavior_manager
# Here is implemented the state machine that controls the switch between the behaviours of the robot
#
# A finite-state machine (FSM) is a behavior model that consists of a finite number of states. 
# Based on the current state and a given input the machine performs state transitions and produces outputs
# The state machine is implemented using the smach library
# It implements four state, Normal, Sleep, Play and Find, and two sub state Normal Track and Find track

import roslib
import rospy
from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Int8
from std_msgs.msg import Float64

import smach
import smach_ros
import time
import random
from final_assignment.msg import Num
import random
import datetime
import subprocess
import signal

from actionlib import GoalID
import cv2
import sys
import time
import numpy as np
from scipy.ndimage import filters
import imutils
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist


## define colours limits for OpenCV algorithm
# in our simulation we have different colourd balls: green, black, red, yellow, blue, magenta
# *** green ***
greenLower = (50, 50, 20)
greenUpper = (70, 255, 255)
# *** black ***
blackLower = (0, 0, 0)
blackUpper = (5, 50, 50)
# *** red ***
redLower = (0, 50, 50)
redUpper = (5, 255, 255)
# *** yellow ***
yellowLower = (25, 50, 50)
yellowUpper = (35, 255, 255)
# *** blue ***
blueLower = (100, 50, 50)
blueUpper = (130, 255, 255)
# *** magenta ***
magentaLower = (125, 50, 50)
magentaUpper = (150, 255, 255)


## function user_action
#
# we use it to switch behavior
def user_action(data):
	# play state
	if(data=='PLAY'):
		return ('play')
	# sleep state
	elif(data=='SLEEP'):
		return ('sleep')
	# normal state
    	elif(data=='NORMAL'):
		return ('normal')
	# find state
    	elif(data=='FIND'):
		return ('find')

## function random_position
#
# function to compute a random goal position
def random_position():
	randomlist = []
	for i in range(0,2):
		n = random.randint(-6,8)
		randomlist.append(n)
   	return randomlist


## class Normal_behavior
#
# This class implement the NORMAL behaviour of the robot pet
# The robot moves randomly within the Gazebo arena
# - If it receives a "play" command the FSM should go into PLAY state (start_play)
# - If the sleep timer is triggered the FSM should go into SLEEP state (start_sleep)
class Normal(smach.State):

	## method init
	#
	# This method should:
	# 	- initializes the state class
    	def __init__(self):

		self.counterSleep = 0
		self.counterPlay = 0
		self.counter = 0
		self.stopFlag = 1
		self.param = []
		self.ball_visible = 'NULL'

		time.sleep(6)
		smach.State.__init__(self, 
		                     outcomes=['play','sleep'])

	## method execute
	#
	# This method:
	# 	- robot moves randomly within the map
	#	- to do so, we sent random goal positions he has to reach
	#	- when it detects a coloured object within the camera range it runs the callback method
	#	- this callback implements a opencv algorithm to detect objects
	# 	- once detected it enters TRACK substate to get closer to the ball and save the room position
    	def execute(self,userdata):
        	rospy.loginfo('******* NORMAL state *******')

		## publishers / subscribers		
		# publisher to topic /goalPos
      		pubGoal = rospy.Publisher('goalPos', Num, queue_size = 10)
		
		# topic /cmd_vel
		# subscriber to check if robot is moving or if it is still
		subVel = rospy.Subscriber("cmd_vel", Twist, self.robot_vel)
		# publisher to topic /cmd_vel
		self.pubVel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

		# subscriber the camera topic
		subCam = rospy.Subscriber("camera1/image_raw/compressed",
		                                   CompressedImage, self.callback,  queue_size=1)

		self.var='FALSE'

		while(self.var=='FALSE'):

			## send the robot to a random goal positions
			# compute random position with function random_position()
			randomlist = random_position()
			rospy.loginfo('*** NORMAL STATE *** Computing position')
			rospy.loginfo('*** NORMAL STATE *** Robot is now moving to: %s', randomlist)
	
			# publish goal to be reach		
			pubGoal.publish(randomlist)
			time.sleep(4)

			#stop sending commands when the robot is moving
	 		while(self.stopFlag == 0):
				pass
			
			## counters to switch behavior
			self.counterPlay = self.counterPlay + 1
			self.counterSleep = self.counterSleep + 1

 			# after some actions have been executed go to SLEEP state
			if self.counterSleep >= 6:
				self.counterSleep = 0 # reinit to 0
				subCam.unregister()
				subVel.unregister()
				return user_action('SLEEP')
			
			# after some actions have been executed enters PLAY state
			# this is used to simulate human command
			if self.counterPlay >= 3 :
				self.counterPlay = 0 # reinit to 0
				subCam.unregister()
				subVel.unregister()
			 	return user_action('PLAY')	

    	## function robot_vel
	#
	# callback to check if the robot is moving or not 
	# A flag is set to 1 if the robot is not moving, to 0 otherwise	 
    	def robot_vel(self, msg):
		if(msg.linear.x == 0.0 and msg.angular.z == 0.0):
			self.stopFlag = 1
		else:
			self.stopFlag = 0	
	## method callback
	#
	# Execute callback everytime e new image is available from the camera. 
	# it make uses of openCV libraries to track the ball within rooms and save the room position
	# it calls methods:
	# 	- countour_opencv
	# 	- sub_track
	def callback(self,ros_data):
 		## cv2 image conversion
		np_arr = np.fromstring(ros_data.data, np.uint8)
		# the function imdecode compresses the image and stores it in memory buffer         					
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
		# Blurs image using a Gaussian filter.
		blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
		# convert image from one color space to another.
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		
		# ****** GREEN ******
		# see definition of method countour_opencv
		self.countour_opencv(greenLower,greenUpper,hsv,image_np)
		
		# ****** BLACK ******
		self.countour_opencv(blackLower,blackUpper,hsv,image_np)
		
		# ****** RED ******
		self.countour_opencv(redLower,redUpper,hsv,image_np)
		
		# ****** YELLOW ******
		self.countour_opencv(yellowLower,yellowUpper,hsv,image_np)
		
		# ****** BLUE ******
		self.countour_opencv(blueLower,blueUpper,hsv,image_np)
		
		# ****** MAGENTA ******
		self.countour_opencv(magentaLower,magentaUpper,hsv,image_np)
	
	## method countour_opencv
	#
	# used to create masks for colours, and their countours
	def countour_opencv(self,lower,upper,hsv,image_np):
		
		# check which colour
		if lower == greenLower and upper == greenUpper : 
			room = 'RoomGreen'
			colour = 'Green'
		if lower == blackLower and upper == blackUpper : 
			room = 'RoomBlack'
			colour = 'Black'
		if lower == redLower and upper == redUpper : 
			room = 'RoomRed'
			colour = 'Red'
		if lower == yellowLower and upper == yellowUpper : 
			room = 'RoomYellow'
			colour = 'Yellow'
		if lower == blueLower and upper == blueUpper : 
			room = 'RoomBlue'
			colour = 'Blue'
		if lower == magentaLower and upper == magentaUpper : 
			room = 'RoomMagenta'
			colour = 'Magenta'

		# create mask for all colours
		mask = cv2.inRange(hsv, lower, upper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		                        cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)

		center = None

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			self.param = rospy.get_param(room)
			if (self.param[2] == 'T'):
				return
			rospy.loginfo('*** TRACK sub-state *** %s ball detected', colour)
			# save which ball 
			self.ball_visible = self.param[3]
			
			## call the sub_track function
			self.sub_track(cnts, image_np, self.ball_visible)
			return		

	
	## method sub_track
	#
	# This function is executed everytime the robot detects a color object which was not previously detected.	
    	# The robot moves closer to the detected object. Once done, the reference position of the room is
    	# marked as reached and stored as "visited" in the parameter server.
    	def sub_track(self,cnts, image_np, ball_visible):

		#rospy.loginfo('******* TRACK substate ******')
		self.counter = self.counter+1
		c = max(cnts, key = cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            	# only proceed if the radius meets a minimum size
            	if radius > 10:
                	# draw the circle and centroid on the frame,
                	# then update the list of tracked points
                	cv2.circle(image_np, (int(x), int(y)), int(radius),
                	           (0, 255, 255), 2)
                	cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                	vel = Twist()
                	vel.angular.z = -0.002*(center[0]-400)
                	vel.linear.x = -0.01*(radius-100)
                	self.pubVel.publish(vel)
			#if the robot is almost not moving register on the parameter server that the corresponding ball has been reached
			if (vel.linear.x < 0.1 and vel.linear.x > -0.1 or self.counter > 250 ):
				self.counter = 0
				# function to store visited room
				self.save_param(ball_visible)
		else:
	                vel = Twist()
	                vel.linear.x = 0.3
	                self.pubVel.publish(vel)

	## method save_param
	#
	# function to store position of the room
	def save_param(self, ball_visible):
		# switch colors
		if(ball_visible=='green_ball'):
			parameter = 'RoomGreen'
		elif(ball_visible=='black_ball'):
			parameter = 'RoomBlack'
		elif(ball_visible=='blue_ball'):
			parameter = 'RoomBlue'
		elif(ball_visible=='red_ball'):
			parameter = 'RoomRed'
		elif(ball_visible=='yellow_ball'):
			parameter = 'RoomYellow'
		elif(ball_visible=='magenta_ball'):
			parameter = 'RoomMagenta'

		self.param= rospy.get_param(parameter)
		self.param[2] = 'T'
		rospy.set_param(parameter, self.param)
		rospy.loginfo('*** TRACK sub-state *** %s position saved', parameter)
		rospy.loginfo('*** Return to NORMAL state ***')
		return

## class Sleep_behavior
#
# This class implement the SLEEP behaviour of the robot pet
# The robot sleeps (SLEEP state)for a random period of time, then it moves to NORMAL state
# The robot should:
#	- reach a predefined location within the arena in Gazebo
#	- stays there for some times 
#	- go back in NORMAL state
class Sleep(smach.State):
	## method init
	#
	# initialization
    	def __init__(self):
        	smach.State.__init__(self, 
        	                     outcomes=['normal'])
		self.stopFlag = 0

    	def execute(self,userdata):
        	rospy.loginfo('****** SLEEP state ******')
		
		# send target position to reach
        	pubGoal = rospy.Publisher('goalPos', Num,queue_size=10) 
	
		# get Home position
		home= rospy.get_param('/homePose')
		lista=[]
		lista.append(home[0])
		lista.append(home[1])
		
		# pub home goal position
		pubGoal.publish(lista)
		rospy.loginfo('*** SLEEP STATE *** Robot is moving to HOME position %s', lista)		
		time.sleep(3)
		
		# check if robot moving 
		while(self.stopFlag==0):
			pass	
		
		#add a sleep to make the robot remain in the sleep state for a certain time
		time.sleep(20)
        
        	return user_action('NORMAL')

    	## method robot_vel
	#
	# callback to check if the robot is moving or not 
	# A flag is set to 1 if the robot is not moving, to 0 otherwise	 	
    	def robot_vel(self, msg):
		if(msg.linear.x < 0.03 and msg.linear.x > -0.03 and msg.angular.z < 0.03 and msg.angular.z > -0.03):
			self.stopFlag=1
		else:
			self.stopFlag=0
               
## class Play_behavior
#
# This class implement the PLAY behavior of the robot pet
# It moves the robot to the predefined (X, Y) location within the map and moves it back to the user.
# The Robot should:
#	- start following the ball
#	- when the ball stops (it means when the robot stops)
#	  it moves the head on the left of 45 degrees, and keep it there for some seconds
#	- then it moves the head to the right, stays there for some seconds
#	- once moved the head keeps tracking the ball
class Play(smach.State):
	## method init
	#
	# initialization
	def __init__(self):	
        	smach.State.__init__(self, 
                	             outcomes=['normal', 'find'],
                	             output_keys=['room_out'])
		self.count = 0
		self.rooms = []
		self.param = []
		self.stopFlag = 0
                          
    	def execute(self,userdata):
		rospy.loginfo('****** PLAY state ******')
		
		## publishers / subscribers
		# publisher to goal position
		pubGoal = rospy.Publisher('goalPos', Num,queue_size=10) 
		# subscriber to /cmd_vel 
		subscriber=rospy.Subscriber("cmd_vel", Twist, self.robot_vel)

		while(self.count < 2):
			self.count = self.count+1

			# get human position
			humanPosition = rospy.get_param('/humanPose')
			# store human Position to be sent as goal 
			lista = []
			lista.append(humanPosition[0])
			lista.append(humanPosition[1])
			pubGoal.publish(lista)
			rospy.loginfo('*** PLAY STATE *** Robot is going back in front of the human')
			rospy.loginfo('*** PLAY STATE *** Robot moving to position: %s', lista)		
			time.sleep(4)
			# wait until the robot stop moving 
			while(self.stopFlag == 0):
				pass
			
			## move to command
			# get array containing the list of all Rooms
			# rooms: ['RoomBlue', 'RoomRed', 'RoomGreen', 'RoomBlack', 'RoomMagenta', 'RoomYellow']
			self.rooms = rospy.get_param('/rooms')	
			# define a random room
			n = random.randint(0,5)
			# store command
			self.goTo = self.rooms[n]
			rospy.loginfo('*** PLAY STATE ***  User command: Move to %s', self.goTo)

			## switch color		
			self.param = rospy.get_param(self.goTo)
			if(self.param[2] == 'T'):
				lista = []
				lista.append(self.param[0])
				lista.append(self.param[1])
				rospy.loginfo('*** PLAY STATE *** Moving to %s in position %s', self.goTo, lista)		
				pubGoal.publish(lista)
			else: 	
				# the robot enters FIND state
				subscriber.unregister()
				userdata.room_out=self.param[3]
				return user_action('FIND')
			time.sleep(2)
			# wait until the robot stop moving 
			while(self.stopFlag==0):
				pass
			time.sleep(5)
		
		subscriber.unregister()
		self.count = 0
		return user_action('NORMAL')	

    	## method robot_vel
	#
	# callback to check if the robot is moving or not 
	# A flag is set to 1 if the robot is not moving, to 0 otherwise		
    	def robot_vel(self, msg):
		if(msg.linear.x == 0 and msg.angular.z == 0):
			self.stopFlag = 1
		else:
			self.stopFlag = 0
	
## class Find_behavior
#
# when entering Find behaviour, the robot moves randomly within the environment
# once it detect a coloured ball, it enters the substate Find Track
class Find(smach.State):
	## method init
	#
	# initialization
    	def __init__(self):
        	smach.State.__init__(self, 
                	             outcomes=['play'],
                	             input_keys=['room_in'])

		self.counter = 0
		self.param = []
		self.stopFlag = 0
		self.play_state = False
		self.child = None

    	def execute(self,userdata):

		rospy.loginfo('****** FIND state ******')
		
		# publisher on topic /cmd_vel
		self.pubVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        	
		# subscribed to the camera topic
        	subscriber = rospy.Subscriber("camera1/image_raw/compressed",
        	                                  CompressedImage, self.callback,  queue_size=1)

		self.requested_room = userdata.room_in
		#launch the explore-lite package everytime the robot enters the play state
		rospy.loginfo('*** FIND STATE *** Launch explore_lite package')		
		self.child = subprocess.Popen(["roslaunch","explore_lite","explore.launch"])
		self.t_final = time.time() + 200

		while(self.play_state == False and time.time() < self.t_final):
      			pass

		time.sleep(2)
		self.child.send_signal(signal.SIGINT)
		subscriber.unregister()
		self.play_state = False
		# return to state play
		return user_action('PLAY')

	## method callback
	#
	# Execute callback everytime e new image is available from the camera. 
	# it make uses of openCV libraries to track the ball within rooms and save the room position
	def callback(self,ros_data):

		## cv2 image conversion
        	np_arr = np.fromstring(ros_data.data, np.uint8)
        	# the function imdecode compresses the image and stores it in memory buffer         					
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  
		# Blurs image using a Gaussian filter.
        	blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
		# convert image from one color space to another.
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)		

		# ****** GREEN ******
		# see definition of method countour_opencv
		self.countour_opencv(greenLower,greenUpper,hsv,image_np)

		# ****** BLACK ******
		self.countour_opencv(blackLower,blackUpper,hsv,image_np)
		
		# ****** RED ******
		self.countour_opencv(redLower,redUpper,hsv,image_np)

		# ****** YELLOW ******
		self.countour_opencv(yellowLower,yellowUpper,hsv,image_np)

		# ****** BLUE ******
		self.countour_opencv(blueLower,blueUpper,hsv,image_np)
	
		# ****** MAGENTA ******
		self.countour_opencv(magentaLower,magentaUpper,hsv,image_np)
 
	## method countour_opencv
	#
	# used to create masks for colours, and their countours
	def countour_opencv(self,lower,upper,hsv,image_np):
		
		# check which colour
		if lower == greenLower and upper == greenUpper : 
			room = 'RoomGreen'
			colour = 'Green'
		if lower == blackLower and upper == blackUpper : 
			room = 'RoomBlack'
			colour = 'Black'
		if lower == redLower and upper == redUpper : 
			room = 'RoomRed'
			colour = 'Red'
		if lower == yellowLower and upper == yellowUpper : 
			room = 'RoomYellow'
			colour = 'Yellow'
		if lower == blueLower and upper == blueUpper : 
			room = 'RoomBlue'
			colour = 'Blue'
		if lower == magentaLower and upper == magentaUpper : 
			room = 'RoomMagenta'
			colour = 'Magenta'

		# create mask for all colours
		mask = cv2.inRange(hsv, lower, upper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		                        cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		center = None

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			self.param = rospy.get_param(room)
			if (self.param[2] == 'T'):
				return
			rospy.loginfo('*** TRACK sub-state *** %s ball detected', colour)
			# save which ball 
			self.ball_visible = self.param[3]
			
			#call the sub_track function
			self.sub_track(cnts, image_np, self.ball_visible)
			return	

	## method sub_track
	#
	# This function is executed everytime the robot detects a color object which was not previously detected.	
    	# The robot moves closer to the detected object. Once done, the reference position of the room is
    	# marked as reached and stored as "visited" in the parameter server.
	def sub_track(self,cnts, image_np, ball_visible):
		# rospy.loginfo('****** TRACK sub-state ******')
	    	self.counter=self.counter+1
            	c = max(cnts, key=cv2.contourArea)
            	((x, y), radius) = cv2.minEnclosingCircle(c)
            	M = cv2.moments(c)
            	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            	# only proceed if the radius meets a minimum size
            	if radius > 10:
            	 	# draw the circle and centroid on the frame,
            	    	# then update the list of tracked points
            	    	cv2.circle(image_np, (int(x), int(y)), int(radius),
            	        	       (0, 255, 255), 2)
            	    	cv2.circle(image_np, center, 5, (0, 0, 255), -1)
		    	vel = Twist()
		    	vel.angular.z = -0.002*(center[0]-400)
		    	vel.linear.x = -0.01*(radius-100)
		    	self.pubVel.publish(vel)
			
			#if the robot is almost not moving register on the parameter server that the corresponding ball has been reached
			if (vel.linear.x<0.1 and vel.linear.x>-0.1 or self.counter>250 ):
				self.counter=0
				# function to store visited room
				self.save_param(ball_visible)
		else:
	                vel = Twist()
	                vel.linear.x = 0.3
	                self.pubVel.publish(vel)
	
	## method save_param
	#
	# function to store position of the room
	def save_param(self, ball_visible):
		# switch colors
		if(ball_visible=='green_ball'):
			parameter = 'RoomGreen'
		elif(ball_visible=='black_ball'):
			parameter = 'RoomBlack'
		elif(ball_visible=='blue_ball'):
			parameter = 'RoomBlue'
		elif(ball_visible=='red_ball'):
			parameter = 'RoomRed'
		elif(ball_visible=='yellow_ball'):
			parameter = 'RoomYellow'
		elif(ball_visible=='magenta_ball'):
			parameter = 'RoomMagenta'

		self.param= rospy.get_param(parameter)
		self.param[2] = 'T'
		rospy.set_param(parameter, self.param)
		rospy.loginfo('*** TRACK sub-state *** %s position saved', parameter)
		rospy.loginfo('*** Return to NORMAL state ***')
		return

## class state_machine
#
# A state machine with 4 states is initialized. 
class state_machine():

	def __init__(self):
                       
    		rospy.init_node('state_machine') 
		# Create a SMACH state machine
		sm = smach.StateMachine(outcomes=['container_interface'])
   

    		# Open the container
    		with sm:
        		# Add states to the container
        		smach.StateMachine.add('NORMAL', Normal(), 
                      			transitions={'play':'PLAY',
						'sleep' : 'SLEEP'})
                                              
        		smach.StateMachine.add('PLAY', Play(), 
               	 			transitions={'normal':'NORMAL',
						    	'find':'FIND'},
				        remapping={'room_out':'sm_room'}) 
                                            
			smach.StateMachine.add('SLEEP', Sleep(), 
                	               	transitions={'normal':'NORMAL'})

			smach.StateMachine.add('FIND', Find(), 
					transitions={'play':'PLAY'},
			       		remapping={'room_in':'sm_room'}) 

    		# Create and start the introspection server for visualization
    		sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    		sis.start()

    		# Execute the state machine
    		outcome = sm.execute()

    		# Wait for ctrl-c to stop the application
    		rospy.spin()
    		sis.stop()
 

if __name__ == '__main__':
    
	state_machine()
