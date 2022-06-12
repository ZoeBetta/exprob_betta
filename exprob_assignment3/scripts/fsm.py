#! /usr/bin/env python

## @package exprob_assignment3
#
#  \file planproblem.py
#  \brief this file implements the calls to the planning server
#
#  \author Zoe Betta
#  \version 1.0
#  \date 21/02/2022
#  \details
#  
#  Subscribes to: <BR>
#	 
#
#  Publishes to: <BR>
#	 
#
#  Services: <BR>
#    
#  Action Services: <BR>
#
#  Client Services: <BR>
#  /rosplan_problem_interface/problem_generation_server
#  /rosplan_planner_interface/planning_server
#  /rosplan_parsing_interface/parse_plan
#  /rosplan_plan_dispatcher/dispatch_plan
#  /rosplan_knowledge_base/update
#    
#
#  Description: <BR>
#  This file implements the logic to generate the plan to control the robot.
#  It reads the feedback from the plan before and keeps generating new plans until
#  all the action of one are  successful. It also updates the knowedge base 
#  depending on a ros parameter to customize the behaviour of the robot.



import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


##
#	\brief This function is called when the node is started
#	\param :
#	\return : None
# 	
#	This function initializes all of the needed services, then it calculates a new plan
#   until the robot is able to fuòòy complete one. 
def main():
    global pub_, active_, act_s
    rospy.init_node('finite_state_machine')
    # initialize the publisher to movebase goal 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    print("server up")
    target=MoveBaseGoal()
    target.target_pose.pose.orientation.w=1
    target.target_pose.header.frame_id="map"
    
    rooms=[ [-4,-3],[-4,2],[-4,7],[5,-7],[5,-3], [5,1]]
    for i in rooms:
	    target.target_pose.pose.position.x=i[0]
	    target.target_pose.pose.position.y=i[1]
	    client.send_goal(target)
	    print("msg sent")
	    client.wait_for_result()
    print ( 'SUCCESS')

if __name__ == '__main__':
    main()
