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
#from exp_assignment2.msg import PlanningAction, PlanningActionGoal
from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Bool # needed for subscribing booleans

publisherPlay = None
publisherCommandRoom = None
at_human = None

## function get_random_position
#
# function to get a random position on the map to move the ball to
def get_room_position():
	rooms = ["Room1", "Room2", "Room3", "Room4", "Room5", "Room6"]
	randPos = rooms[random.randint(0, 5)] # get a random room
	return randPos

## function check_at_human()
#
# callback to check if the robot is in front of the human or not
def check_at_human(human):	
	global at_human
	at_human = human.data

## main function
#
# initialize the node
def main():
	global publisherPlay, publisherCommandRoom, at_human
	#init node
	rospy.init_node("human_simulator")	
	rate = rospy.Rate(20)
	
	# init action client
	publisherPlay = rospy.Publisher("/human_command_play", Bool, queue_size=1)
    	publisherCommandRoom = rospy.Publisher("/room_command", String, queue_size=1)
   
	# subscriber to check if at human
	rospy.Subscriber("/at_human", Bool, check_at_human)
	
	while not rospy.is_shutdown():	
		# random command to enter play
		if random.randint(1,rospy.get_param("frequency_play")) == 1:
			# publish on topic
			publisherPlay.publish(True)
		
		# if in front of human we can send a message	
		if at_human:
			# random time interval
			rospy.sleep(random.randInt(2,4))
			pos = get_room_position()
			rospy.loginfo("NODE HUMAN SIMULATOR: robot should move to %s (%s)", pos, rospy.get_param(pos))
			# pub command
			publisherCommandRoom.publish(pos)
			at_human = False
		rate.sleep()


if __name__ == "__main__":
	main()
