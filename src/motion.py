#!/usr/bin/env python

## @package motion
#
# It moves the robot within the Gazebo environment according to the given behavior

import rospy
#import time
import random
from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Bool # needed for subscribing booleans
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import CompressedImage
import math
import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

VERBOSE = False # flag to make regular expressions more readable

## global variables
# initialize behavior
behaviour = None # it is used to check current behavior
currentRoom = None
currentRoomPos = None

# define coordinates home position 
home_position = [rospy.get_param('home_x'), rospy.get_param('home_y')]
# define goal
goalPos = MoveBaseGoal() # goal position to reach 
# action client initializer
act_c = None
#publisher to home position
publisherAtHome = rospy.Publisher("/at_home", Bool, queue_size=1)
publisherAtHuman = rospy.Publisher("/at_human", Bool, queue_size=1)
publisherUnknownRoom = rospy.Publisher("/unknown_room", Bool, queue_size=1)

# home_reached publisher
at_home = False
at_human = False
rate = None

## function random position on map
#
# get a random position given x and y coordinates
def get_random_position():
    randX = random.randint(0, 3) # x coordinate
    randY = random.randint(0, 3) # y coordinate
    randPos = [randX, randY]
    return randPos

## function get_current_room_position
#
# function to check if the room position is known
def get_current_room_position():
	while(not rospy.is_shutdown()):
		if (currentRoom != None):
        		break
        	rate.sleep()

    	colour = rospy.get_param(currentRoom)
    	if rospy.search_param(colour) != None:
        	currentRoomPos = rospy.get_param(colour)
    	else:
        	currentRoomPos = None
    	return currentRoomPos

## function callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(state):
	#rospy.loginfo('NODE MOTION: Executing callback behavior')
    global behaviour
    behaviour = state.data

## function get_room_command
#
# read the command from human
def get_room_command(human_command):
	global currentRoom
    	currentRoom = human_command.data 
    
## function feedback_cb
#
# callback to send goal
def feedback_cb(check):
	# while the robot is reaching the goal position, check if the behaviour changes
	if behaviour == 'play' or behaviour == 'sleep' or behaviour == 'normal_track':
		rospy.loginfo("NODE MOTION: Warning, changed behavior in %s", behaviour)
		# we need to cancel all goals        
		act_c.cancel_all_goals()  # cancel() 


## function move_random_normal
#
# the robot moves randomly when in the NORMAL state
def move_random_normal():
	rospy.loginfo("NODE MOTION: Execute function to move randomly when in NORMAL mode")
	
	# compute random position
	randPos = get_random_position()

	goalPos.target_pose.header.frame_id = "map"
    	goalPos.target_pose.header.stamp = rospy.Time.now()
	# set random position to be reached
	goalPos.target_pose.pose.position.x = randPos[0]
    	goalPos.target_pose.pose.position.y = randPos[1]
    	goalPos.target_pose.pose.position.z = 0
	goalPos.target_pose.pose.orientation.w = 2
	# send robot position and wait that the goal is reached within 60 seconds
    	act_c.send_goal(goalPos, feedback_cb=feedback_cb) # send position
    	rospy.loginfo("NODE MOTION: the goal position was sent, it corresponds to:")
    	rospy.loginfo(goalPos.target_pose.pose.position)
	# wait for some time
    	act_c.wait_for_result(rospy.Duration.from_sec(180.0))
	result = act_c.get_result()
	if result:
		rospy.loginfo("NODE MOTION: Robot has reached the goal or behaviour has changed")
    	

## function move_sleep_position
#
# movement in the SLEEP state
def move_sleep_position():
	global at_home # used to check if the robot is at home position or not	
	
	goalPos.target_pose.header.frame_id = "map"
	goalPos.target_pose.header.stamp = rospy.Time.now()

	## set robot goal position
	goalPos.target_pose.pose.position.x = home_position[0]
    	goalPos.target_pose.pose.position.y = home_position[1]
    	goalPos.target_pose.pose.position.z = 0
	goalPos.target_pose.pose.orientation.w = 2
	
	# send robot position and wait that the goal is reached within 60 seconds
	act_c.send_goal(goalPos)
    	rospy.loginfo("NODE MOTION: Robot returns home...")
    	rospy.loginfo(goalPos.target_pose.pose.position)
    	act_c.wait_for_result(rospy.Duration.from_sec(240.0))
    	result = act_c.get_result()
    	if result:
        	rospy.loginfo("NODE MOTION: Robot has reached the home position, it now switches to sleep behaviour")
        	at_home = True

## function play_motion
#
# it control the movements of the robot in play behaviour
def play_motion():
	global at_human, currentRoom
		
	if not at_human:
		goalPos.target_pose.header.frame_id = "map"
        	goalPos.target_pose.header.stamp = rospy.Time.now()
		# set robot goal position
        	goalPos.target_pose.pose.position.x = -5
        	goalPos.target_pose.pose.position.y = 8
        	goalPos.target_pose.pose.position.z = 0
        	goalPos.target_pose.pose.orientation.w = 2
		# send robot position and wait that the goal is reached within 60 seconds
        	act_c.send_goal(goalPos)
        	rospy.loginfo("NODE MOTION: Robot goes back to where the human is")
        	act_c.wait_for_result(rospy.Duration.from_sec(240.0))
        	result = act_c.get_result()
		if result:
			rospy.loginfo("NODE MOTION: Robot has reached the human position in time")
		    	at_human = True
		    	currentRoomPos = None
		    	publisherAtHuman.publish(at_human)
	else:
        	currentRoomPosition = get_current_room_position()
        	# if the room position is unknown, publish to 'no_room' topic (go to Find behaviour)
        	if currentRoomPosition == None:
        		rospy.loginfo("NODE MOTION: the room to reach is unknown!")
        
            		# signal that the location is not known
            		publisherUnknownRoom.publish(True)
            		publisherAtHuman.publish(False)
            		rospy.sleep(2)
            		at_human = False
            		currentRoom = None
        	# in case the room was already mapped, then the robot can reach it
        	elif currentRoomPosition != None:
        		goalPos.target_pose.header.frame_id = "map"
        		goalPos.target_pose.header.stamp = rospy.Time.now()
        		# set room goal position
            		goalPos.target_pose.pose.position.x = currentRoomPosition[0]
            		goalPos.target_pose.pose.position.y = currentRoomPosition[1]
            		goalPos.target_pose.pose.position.z = 0
            		goalPos.target_pose.pose.orientation.w = 2
            		# read current position, send it to the robot and wait for him to reach it
            		act_c.send_goal(goalPos)
            		rospy.loginfo("NODE MOTION: the robot going to the %s (%s)", currentRoom, rospy.get_param(currentRoom))
            		act_c.wait_for_result(rospy.Duration.from_sec(240.0))
            		result = act_c.get_result()
            		if result:
                		rospy.loginfo("NODE MOTION: the robot has reached the room %s (%s) in time", currentRoom, rospy.get_param(currentRoom))
                		# wait some time before returning to the human
                		rospy.sleep(random.randint(3,10))
                		at_human = False
                		currentRoom = None
                		currentRoomPosition = None
                

## function main
#
def main():
	## initialize node
	rospy.init_node("motion")
	global at_home # boolean to check if robot at home or not
	global at_human # boolean to check if robot is in front of the human or not
	global act_c # goal position to reach 
	global rate

    	rate = rospy.Rate(20)

    	# subscriber to go_to command
    	rospy.Subscriber("/room_command", String, get_room_command)

    	# subscriber to current behaviour
    	rospy.Subscriber("/behavior", String, callback_get_behaviour)
    	rospy.loginfo("NODE MOTION: subscribed to the behaviour")

	# initialize action client move_base package
	act_c = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	
	# wait for the initialization of the server for some seconds
    	act_c.wait_for_server(rospy.Duration(10))

	## move according to the behaviour
	while not rospy.is_shutdown():			
		# check robot behavior
		if behaviour == "sleep":
			## the robot moves to predefined location
			# call function move_sleep_position()
			if not at_home:
				move_sleep_position()
				if at_home:
					publisherAtHome.publish(at_home)
		else:
			# not in goal
			at_home = False
			
			if behaviour == "normal":
                		move_random_normal()
                		rospy.sleep(1)
            
            		if behaviour == "play":
                		play_motion()
			
		rate.sleep()	

if __name__ == "__main__":
	main()
