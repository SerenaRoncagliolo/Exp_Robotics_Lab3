#!/usr/bin/env python

## @package human_simulator
#
# Human interactions with the ball
# The human can:
# 	1) move the ball around
#		- give a command position to the ball
#		- wait for a number of seconds
#		- give another command to the position of the ball
#	2) make the ball disappear
#		- it sends a position with negative z

import rospy
import random
from geometry_msgs.msg import PoseStamped
import actionlib
import actionlib.msg
from exp_assignment2.msg import PlanningAction, PlanningActionGoal

act_c = None
goalPos = PlanningActionGoal()

## function get_random_position
#
# function to get a random position on the map to move the ball to
def get_random_position():
	randX = random.randint(-8, 8)
	randY = random.randint(-8, 8)
	randZ = 0.5
	randPos = [randX, randY, randZ]
	return randPos

## function move_ball()
#
# function to make the ball moving 
def move_ball():
	# random position
	#randX = random.randint(-7, 7)
	#randY = random.randint(-7, 7)
	#randZ = 0.5
	#randPos = [randX, randY, randZ]
	randPos = get_random_position()

    	# set ball goal position 
   	goalPos.goal.target_pose.pose.position.x = randPos[0]
    	goalPos.goal.target_pose.pose.position.y = randPos[1]
    	goalPos.goal.target_pose.pose.position.z = randPos[2]

    	# send ball position and wait that the goal is reached within 60 seconds
    	act_c.send_goal(goalPos.goal)
    	act_c.wait_for_result(rospy.Duration.from_sec(60.0))

## function make_disappear_ball()
#
# function to send command to make the ball disappear
def make_disappear_ball():	
	# random position
    	randPos = get_random_position()

	# move the ball under the map
	goalPos.goal.target_pose.pose.position.x = 0
	goalPos.goal.target_pose.pose.position.y = 0
	goalPos.goal.target_pose.pose.position.z = -1

	# send position and wait that the goal is reached within 60 seconds
	act_c.send_goal(goalPos.goal)
	act_c.wait_for_result(rospy.Duration.from_sec(60.0))



## main function
#
# initialize the action client and move the ball
def main():
	#init node
	rospy.init_node("human_simulator")	
	rate = rospy.Rate(20)
	
	# init action client
	global act_c
	act_c = actionlib.SimpleActionClient('/ball/reaching_goal', PlanningAction)
   	
	# wait for the initialization of the server
   	act_c.wait_for_server(rospy.Duration(5))
	
	while not rospy.is_shutdown():
		# random choice
		if random.randint(1,4) == 1:
			# make the ball disappear
			make_disappear_ball()
		else:
			move_ball()

		# wait some random time
		rospy.sleep(random.randint(7,10))

		rate.sleep()


if __name__ == "__main__":
	main()
