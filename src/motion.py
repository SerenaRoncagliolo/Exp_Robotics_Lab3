#!/usr/bin/env python

## @package motion
#
# It moves the robot within the Gazebo environment according to the given behavior

import rospy
import time
import random
import math
import actionlib
import actionlib.msg

from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Bool # needed for subscribing booleans
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import CompressedImage
from exp_assignment2.msg import PlanningAction, PlanningActionGoal

VERBOSE = False # flag to make regular expressions more readable

## global variables
# initialize behavior
behaviour = None # it is used to check current behavior
# define coordinates home position 
home_position = [rospy.get_param('home_x'), rospy.get_param('home_y')]
# define goal
goalPos = PlanningActionGoal() # goal position to reach [x,y]
# action client initializer
act_c = None
#publisher to home poasiotion
publisherHome = rospy.Publisher("/at_home", Bool, queue_size=1)
# home_reached publisher
at_home = False

## function random position on map
#
# get a random position given x and y coordinates
def get_random_position():
    randX = random.randint(-7, 7) # x coordinate
    randY = random.randint(-7, 7) # y coordinate
    randPos = [randX, randY]
    return randPos

## function callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	#rospy.loginfo('NODE MOTION: Executing callback behavior')
	global behaviour 
	behaviour = data.data

## function feedback_cb
#
# callback to send goal
def feedback_cb(check):
	robot_in_target = False
	if check.stat == "Target position reached":
		robot_in_target = True
	# while the robot is in goal position, check if the behaviour changes
	if behaviour == 'play' or behaviour == 'sleep':
		rospy.loginfo("NODE MOTION: Warning, changed behavior")
		# we need to cancel all goals        
		act_c.cancel_all_goals()  # cancel() 
	elif robot_in_target:
		# if the goal has been reached
		rospy.loginfo("NODE MOTION: Goal reached ")


## function move_random_normal
#
# the robot moves randomly when in the NORMAL state
def move_random_normal():
	rospy.loginfo("NODE MOTION: Execute function to move randomly when in NORMAL mode")
	
	# compute random position
	randPos = get_random_position()

	# set random position to be reached
	goalPos.goal.target_pose.pose.position.x = randPos[0]
    	goalPos.goal.target_pose.pose.position.y = randPos[1]
    	goalPos.goal.target_pose.pose.position.z = 0
	    
	# send robot position and wait that the goal is reached within 60 seconds
    	act_c.send_goal(goalPos.goal, feedback_cb=feedback_cb) # send position
    	rospy.loginfo("NODE MOTION: the goal position was sent, it corresponds to:")
    	rospy.loginfo(goalPos.goal.target_pose.pose.position)
	# wait for some time
    	act_c.wait_for_result(rospy.Duration.from_sec(50.0))
    	

## function move_sleep_position
#
# movement in the SLEEP state
def move_sleep_position():
	
	global at_home # used to check if the robot is at home position or not
	## set home position
	goalPos.goal.target_pose.pose.position.x = home[0]
    	goalPos.goal.target_pose.pose.position.y = home[1]
    	goalPos.goal.target_pose.pose.position.z = 0

	# send robot position and wait that the goal is reached within 60 seconds
	act_c.send_goal(goalPos.goal)
	rospy.loginfo("NODE MOTION: goal position coordinates sent sent")
	rospy.loginfo(goalPos.goal.target_pose.pose.position)
	act_c.wait_for_result(rospy.Duration.from_sec(60.0))
	rospy.loginfo("NODE MOTION: robot has reached goal position")
	home_reached = True

## main function
#
def main():
	## initialize node
	rospy.init_node('motion')
	
	global at_home # boolean to check if robot at home or not
	global act_c # goal position to reach 
	
	
	## subscriber
	rospy.loginfo('NODE MOTION: Subscriber to /behavior topic')
	rospy.Subscriber("/behavior", String, callback_get_behaviour)		

	# initialize action client
	act_c = actionlib.SimpleActionClient('/robot/reaching_goal', PlanningAction)
	
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
			move_random_normal()
			rospy.sleep(1)
	

if __name__ == "__main__":
	main()
