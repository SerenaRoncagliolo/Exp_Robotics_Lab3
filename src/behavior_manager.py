#!/usr/bin/env python

## @package behavior_manager
# Here is implemented the state machine that controls the switch between the behaviours of the robot
#
# A finite-state machine (FSM) is a behavior model that consists of a finite number of states. 
# Based on the current state and a given input the machine performs state transitions and produces outputs
# The state machine is implemented using the smach library
# It implements four state, Normal, Sleep, Play and Find, and two sub state Normal Track and Find track

import os
import rospy
import smach
import smach_ros
#import time
import random
import roslaunch # to launch node within the code
import actionlib

from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Bool # needed for subscribing booleans
from move_base_msgs.msg import MoveBaseAction # messages from package move_base

## publisher pub_behavior
#
# the node publishes on the behavior topic using a message of type String.
# the queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
pub_behavior = rospy.Publisher("/behavior", String, queue_size=1)


## class Normal_behavior
#
# This class implement the NORMAL behaviour of the robot pet
# The robot moves randomly within the Gazebo arena
# - If it receives a "play" command the FSM should go into PLAY state (start_play)
# - If the sleep timer is triggered the FSM should go into SLEEP state (start_sleep)
class Normal_behavior(smach.State):
	## method init
	#
	# This method should:
	# 	- initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
		                     outcomes=['start_sleep','start_play','start_track']
		                    )	
		 # Loop 100Hz
		self.rate = rospy.Rate(20) # loop at freq 20 Hz

	## method execute
	#
	# - publish "normal" (String) on the topic behavior
	# - moves the robot randomly
	# - if a command from the human is received
	# 	- the robot switches to Play state
	# - if the ball is visible:
	#	- the robot switches to Normal Track sub state
	def execute(self, userdata):
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish normal behavior")
		pub_behavior.publish("normal") 

		self.human_command_play = False
		self.ball_visible = False
		
		## check if the ball is visible to the robot
		rospy.Subscriber("/ball_visible", Bool, self.ball_tracking)
		## check if the robot should switch to Play behaviour
		rospy.Subscriber("/human_command_play", Bool, self.read_command)

		# seconds counter
		seconds_counter = 0

		# get current time to compute a random interval from the istant in which the robot entered normal state
		start_time = rospy.Time.now()
		
		while not rospy.is_shutdown():
			
			## compute how long the robot stays in Normal state 
			if seconds_counter == 1: 
				start_time = rospy.Time.now()
			# update counter
			seconds_counter += 1			
			current_time = rospy.Time.now()
			# compute how long it stays in normal time
			tot_time_given = current_time.secs - start_time.secs
			# if ball detected
			if(self.ball_visible):
				## the robot sees the ball it should enter sub-state track
				return 'start_track'
			# if command from human
			if(self.human_command_play):
				## if the play command is received the robot should switch to play
				return 'start_play'
			elif(random.randint(1,rospy.get_param("frequency_sleep")) == 1 and tot_time_given > 10):
				## the robot goes to sleep at random time
				return 'start_sleep'

			self.rate.sleep()				
			
	## method ball_tracking
	#
	# method to check if the ball is visible to the robot or not
	def ball_tracking(self, ball):
		self.ball_visible = ball.data
	## method read_command
	#
	# method to check if if received a command or not
	def read_command(self, human_command):
		self.human_command_play = human_command.data


## class Normal_track_behaviour
#
# substate of the Normal state. When the robot sees a ball while in Normal state
# it tries to move closer and store its position
class Normal_track_behaviour(smach.State):
	## method init
	#
	# state initialization
	def __init__(self):
		smach.State.__init__(self, outcomes=['start_normal'])
		self.rate = rospy.Rate(20) # loop

	## method execute
	#
	# once the robot has moved closer to the ball, the function check if it
	# is really close to the ball or not
	def execute(self, userdata):
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish normal track behavior")
		pub_state.publish("normal_track")
		self.ball_reached = False

		rospy.Subscriber("/at_ball", Bool, self.check_at_ball)

		while not rospy.is_shutdown():
			if(self.at_ball):
				# if the robot has already reached it -> goes into normal 
				return "start_normal"
			self.rate.sleep()

	# method check_at_ball
	#
	# subscriber callback for ball reaching
	def check_at_ball(self, check):
		self.at_ball = check.data		

## class Find_track_behaviour
#
# substate of the robot from Find behaviour
class Find_track_behaviour(smach.State):
	## method init
	#
	# initialization
	def __init__(self):
		smach.State.__init__(self, outcomes = ['start_find', 'start_play'])		
		self.rate = rospy.Rate(20) # loop
	
	## method execute
	#
	# state execution
	def execute(self, userdata):
		rospy.loginfo("NODE BEHAVIOR_MANAGER: entering sub-state Find Track")
		pub_state.publish("find_track")
		self.at_ball = False
		self.at_room = False
		
		# check if the robot has reached the correct room
		rospy.Subscriber("/at_ball", Bool, self.check_at_ball)
		rospy.Subscriber("/at_room", Bool, self.check_at_room)

		while not rospy.is_shutdown():
			# we check if the robot is in the correct room
			# if not it should keep searchinng
			if(self.at_ball and not self.at_room):
				return 'start_find'
			elif(self.at_ball and self.at_room):
				return 'start_play'
			self.rate.sleep()
	## method check_at_ball
	#
	# callback to check if the robot is close at the detected ball
	def check_at_ball(self, ball):
		self.at_ball = ball.data
	## method check_at_room
	#
	# callback to check if the robot is close at the detected ball
	def check_at_room(self, room):
		self.at_room = room.data


			
## class Sleep_behavior
#
# This class implement the SLEEP behaviour of the robot pet
# The robot sleeps (SLEEP state)for a random period of time, then it moves to NORMAL state
# The robot should:
#	- reach a predefined location within the arena in Gazebo
#	- stays there for some times 
#	- go back in NORMAL state
class Sleep_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
                             outcomes=['stop_sleep']
                            )
		#self.at_home = False # initialize boolean variable to check if at home or not
		self.rate = rospy.Rate(20)
		
	## method execute
	#
	# This method should
	# - publish "sleep" (String) on the topic behavior
	# - check if the robot is already at home position or not
	def execute(self, userdata):
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish sleep behavior")
		pub_behavior.publish("sleep") 
		
		self.at_home = False
		# when "sleep" is published on the topic behavior, the node motion should 
		# subscribe to it and send the robot at the home position
		# it now subscribe to Motion to check if it is at home
		# to check if it's already at home position we read the actual position of the robot
		rospy.Subscriber("/at_home", Bool, self.check_if_home_position)
		# we call the callback read_actual_position to save the values in self.position 
		while not rospy.is_shutdown():  
			# check if it is in home position
			if(self.at_home):
				# make it sleep for some time, rospy.sleep()
				rospy.loginfo('NODE BEHAVIOR: Wait some time to wake up')
				rospy.sleep(random.randint(20,40))
				self.at_home = False
				# exit sleep state
				return 'stop_sleep'
		self.rate.sleep()
	## method read_actual_position
	#
   	# subscriber to actual_position_robot topic callback, it reads the actual position of the robot
   	def check_if_home_position(self, home):
		self.at_home = home.data

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

class Play_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
		                     outcomes=['stop_play','start_find']
		                    )
		self.rate = rospy.Rate(20)  

	## method execute
	#
	# what the robot should do
	def execute(self, userdata):
		# publish behavior "play" on the topic \behavior
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish play behavior")
		pub_behavior.publish("play") 

		self.room_unknown = False
		# if the room was not visited before, it is still unkown to the robot
		rospy.Subscriber("/unknown_room", Bool, self.read_current_location)
		
		# seconds counter
		seconds_counter = 0		
		# get current time to compute a random interval from the istant
		# in which the robot entered normal state
		start_time = rospy.Time.now()

		while not rospy.is_shutdown():
			## compute how long the robot stays in Normal state 
			if seconds_counter == 1: 
				start_time = rospy.Time.now()
			# update counter
			seconds_counter += 1
			current_time = rospy.Time.now()
			# compute how long it stays in normal time
			tot_time_given = current_time.secs - start_time.secs

			if self.room_unkown:
				## if the location sent by the human is unknown go to Find state
				return 'start_find'
			if(tot_time_given > random.randint(120,360)):
				## after 2-6 minutes return to Normal State
				return 'stop_play'
			self.rate.sleep()
				
	## method read_ball_detection
	#
	# subscriber callback to find the ball 
	def read_current_location(self,currentRoom):
		self.room_unknown = currentRoom.data


## class Find_behavior
#
# when entering Find behaviour, the robot moves randomly within the environment
# once it detect a coloured ball, it enters the substate Find Track
class Find_behavior(smach.State):
	## method init
	#
	# initialization
	def __init__(self):
		smach.State.__init__(self, outcomes=['start_play', 'start_track'])
		self.rate = rospy.Rate(20)

		# create action client to move_base and wait for the server answer
		self.movebaseClient = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		self.movebaseClient.wait_for_server()
	## method execute
	#
	# state execution
	def execute(self,userdata):
		rospy.loginfo("NODE BEHAVIOR_MANAGER: publish find behavior")
		pub_state.publish("find")
			
		self.ball_detected = False
		# subscriber for ball detection
		rospy.Subscriber("/ball_visible", Bool, self.ball_tracking)

		# use roslaunch to execute the ros explore_lite package
		# package = 'explore_lite'
		# node_type = 'explore'
		# node_name = 'explore'
		node = roslaunch.core.Node('explore_lite', 'explore', 'explore')

		launch = roslaunch.scriptapi.ROSlaunch()
		launch.start()

		process = launch.launch(node)
		
		# seconds counter
		seconds_counter = 0
		
		# get current time to compute a random interval from the istant
		# in which the robot entered normal state
		start_time = rospy.Time.now()

		while not rospy.is_shutdown():
			## compute how long the robot stays in Normal state 
			if seconds_counter == 1: 
				start_time = rospy.Time.now()
			# update counter
			seconds_counter += 1
			current_time = rospy.Time.now()
			# compute how long it stays in normal time
			tot_time_given = current_time.secs - start_time.secs

			# after some time go back to Play state
			if(tot_time_given > random.randint(240,420)):
				# stop explore-lite
				self.movebaseClient.cancel_all_goals()
				process.stop()
				if not process.is_alive():
					rospy.loginfo("NODE BEHAVIOUR MANAGER: Stop explore_lite, enter Play state")
				
				return "start_play"	

			# if the robot detects the ball then it enter the sub-state track
			if self.ball_visible:
				# stop package explore-lite
				self.movebaseClient.cancel_all_goals()
				process.stop()
				if not process.is_alive():
					rospy.loginfo("NODE BEHAVIOUR MANAGER: Stop explore_lite, enter Track sub-state")
				
				return 'start_track'
			
			# loop
			self.rate.sleep()
		## method ball_tracking
		#
		# subscriber call
		def ball_tracking(self, ball):
			self.ball_visible = ball.data




## function main
#
# state machine 
def main():
	rospy.init_node("behavior_manager")
	
	# initialization of the sys
	rospy.sleep(2)	

	## Create state machine	
	sm = smach.StateMachine(outcomes=['container_interface'])
	
	## machine container
	with sm:
		## add states to the container,
		smach.StateMachine.add('NORMAL', Normal_behavior(), transitions={'start_sleep':'SLEEP','start_play':'PLAY', 'start_track':'NORMALTRACK'})
		smach.StateMachine.add('SLEEP', Sleep_behavior(), transitions={'stop_sleep':'NORMAL'})	
 		smach.StateMachine.add('PLAY', Play_behavior(), transitions={'stop_play':'NORMAL','start_find':'FIND'})	
		smach.StateMachine.add('FIND', Find_behavior(), transitions={'start_play':'PLAY', 'start_track':'FINDTRACK'})	
		smach.StateMachine.add('NORMALTRACK', Normal_track_behaviour(), transitions={'start_normal':'NORMAL'})	
		smach.StateMachine.add('FINDTRACK', Find_track_behaviour(), transitions={'start_find':'FIND', 'start_play':'PLAY'})	
	
	## Create and start the introspection server for visualization
   	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    	sis.start()

	# Execute SMACH plan
	outcome = sm.execute()
		
	## Wait for ctrl-c to stop the application
    	rospy.spin()
    	sis.stop()


if __name__ == "__main__":
	main()

