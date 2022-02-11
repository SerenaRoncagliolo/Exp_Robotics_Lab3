#!/usr/bin/env python

## @package motion
#
# It moves the robot within the Gazebo environment according to the given behavior

import rospy
#import time
import random
import math
import actionlib
import actionlib.msg

from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Bool # needed for subscribing booleans
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import CompressedImage

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
publisherAtHome= rospy.Publisher("/at_home", Bool, queue_size=1)
publisherAtHuman= rospy.Publisher("/at_human", Bool, queue_size=1)
publisherUnknownRoom= rospy.Publisher("/unknown_room", Bool, queue_size=1)
# home_reached publisher
at_home = False
at_human = False
rate = None
## function random position on map
#
# get a random position given x and y coordinates
def get_random_position():
    randX = random.randint(-6, 6) # x coordinate
    randY = random.randint(-8, 8) # y coordinate
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
        currentRoomPosition = rospy.get_param(colour)
    else:
        currentRoomPosition = None
    return room_pos
## function callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	#rospy.loginfo('NODE MOTION: Executing callback behavior')
	global behaviour 
	behaviour = data.data

## function get_room_command
#
# read the command from human
def get_room_command(human_command):
    global curretnRoom
    curretnRoom = human_command.data 
    

## function feedback_cb
#
# callback to send goal
def feedback_cb(check):
	# while the robot is reaching the goal position, check if the behaviour changes
	if behaviour == 'play' or behaviour == 'sleep' or behaviour == 'track_normal':
		rospy.loginfo("NODE MOTION: Warning, changed behavior")
		# we need to cancel all goals        
		act_c.cancel_all_goals()  # cancel() 


## function move_random_normal
#
# the robot moves randomly when in the NORMAL state
def move_random_normal():
	rospy.loginfo("NODE MOTION: Execute function to move randomly when in NORMAL mode")
	
	# compute random position
	randPos = get_random_position()

	goal_pos.target_pose.header.frame_id = "map"
    goal_pos.target_pose.header.stamp = rospy.Time.now()
	# set random position to be reached
	goalPos.goal.target_pose.pose.position.x = randPos[0]
    goalPos.goal.target_pose.pose.position.y = randPos[1]
    goalPos.goal.target_pose.pose.position.z = 0
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
	goaléos.target_pose.header.stamp = rospy.Time.now()

	## set robot goal position
	goalPos.goal.target_pose.pose.position.x = home[0]
    goalPos.goal.target_pose.pose.position.y = home[1]
    goalPos.goal.target_pose.pose.position.z = 0
	goal_pos.target_pose.pose.orientation.w = 2
	
	# send robot position and wait that the goal is reached within 60 seconds
	act_c.send_goal(goalPos.goal)
	act_c.send_goal(goal_pos)
    rospy.loginfo("Robot returns home...")
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
	global at_home, currentRoom
		
	if not at_human:
		goalPos.target_pose.header.frame_id = "map"
        goalPos.target_pose.header.stamp = rospy.Time.now()
		# set robot goal position
        goalPos.target_pose.pose.position.x = home[0]
        goalPos.target_pose.pose.position.y = home[1]
        goalPos.target_pose.pose.position.z = 0
        goalPos.target_pose.pose.orientation.w = 2
		# send robot position and wait that the goal is reached within 60 seconds
        act_c.send_goal(goal_pos)
        rospy.loginfo("NODE MOTION: Robot goes back to where the human is")
        act_c.wait_for_result(rospy.Duration.from_sec(240.0))
        result = act_c.get_result()
        if result:
            rospy.loginfo("NODE MOTION: Robot has reached the human position in time")
            at_human = True
            currentRoomPos = None
            pub_human_reached.publish(at_human)
    else:
        currentRoomPosition = get_room_pos()
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
        elif room_position != None:
            goal_pos.target_pose.header.frame_id = "map"
            goal_pos.target_pose.header.stamp = rospy.Time.now()
            # set room goal position
            goal_pos.target_pose.pose.position.x = room_position[0]
            goal_pos.target_pose.pose.position.y = room_position[1]
            goal_pos.target_pose.pose.position.z = 0
            goal_pos.target_pose.pose.orientation.w = 2
            # read current position, send it to the robot and wait for him to reach it
            act_c.send_goal(goal_pos)
            rospy.loginfo("NODE MOTION: the robot going to the %s (%s)", room, rospy.get_param(room))
            #rospy.loginfo(goal_pos.target_pose.pose.position)
            act_c.wait_for_result(rospy.Duration.from_sec(240.0))
            result = act_c.get_result()
            if result:
                rospy.loginfo("NODE MOTION: the robot has reached the room %s (%s) in time", room, rospy.get_param(room))
                # wait some time before returning to the human
                rospy.sleep(random.randint(3,10))
                at_human = False
                currentRoom = None
                currentRoomPosition = None
                

## function main
#
def main():
	## initialize node
	rospy.init_node('motion')
	global at_home # boolean to check if robot at home or not
	global at_human # boolean to check if robot is in front of the human or not
	global act_c # goal position to reach 

    rate = rospy.Rate(20)

    # subscriber to go_to command
    rospy.Subscriber("/room_command", String, get_room_command)

    # subscriber to current behaviour
    rospy.Subscriber("/behaviour", String, get_behaviour)
    rospy.loginfo("NODE MOTION: subscribed to the behaviour")

	# initialize action client move_base package
	act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	
	# wait for the initialization of the server for some seconds
    act_c.wait_for_server(rospy.Duration(10))

	## move according to the behaviour
	while not rospy.is_shutdown():			
		# check robot behavior
		if(behaviour == "sleep"):
			## the robot moves to predefined location
			# call function move_sleep_position()
			if not at_home:
				move_sleep_position()
				if at_home:
					publisherHome.publish(at_home)
		else:
			# not in goal
			at_home = False
			
			if behaviour == "normal":
                move_random_normal()()
                rospy.sleep(1)
            
            if behaviour == "play":
                move_play()
			
		rate.sleep()	
if __name__ == "__main__":
	main()
