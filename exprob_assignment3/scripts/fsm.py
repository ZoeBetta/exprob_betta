#! /usr/bin/env python

## @package exprob_assignment3
#
#  \file fsm.py
#  \brief this file implemets the finite state machine
#
#  \author Zoe Betta
#  \version 1.0
#  \date 16/06/2022
#  \details
#  
#  Subscribes to: <BR>
#	/complete_found 
#   /complete
#
#  Publishes to: <BR>
#	/cmd_vel 
#
#  Services: <BR>
#    
#  Action Services: <BR>
#   move_base
#
#  Client Services: <BR>
#   /oracle_solution
#   /results
#
#  Description: <BR>
#  This file implements the finite state machine algorithms that controls
#  the behavior of the robot. The robot should move in a random room, once 
#  it reaches the center of the room it starts a looking around behavior
#  with the arm in two different positions. When there is something published 
#  on the topic /complete_found this behavior stops, the robot reaches the
#  center of the arena and calls the server to see if one of the complete 
#  and consistent hypothesis is the  winning one


import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Twist, Point, Pose
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
import moveit_commander
import moveit_msgs.msg
from math import pi
from std_msgs.msg import String, Int32, Bool
from erl2.srv import Oracle
from exprob_assignment3.srv import Results, ResultsRequest

#global variables
finished=0
target=MoveBaseGoal()
client=None
rooms=[ [-4,-3],[-4,2],[-4,7],[5,-7],[5,-3], [5,1]]
visited=[]
counter=0
pub_=None
state=2
move_group =None
up=True
complete_hyp_to_check=[]
complete_hyp_checked=[]
oracle_solution_service=None
result_service=None
complete_available=False


##
#	\brief this function implements the request to move to another room
#	\param : None
#	\return : None
# 	
#   This function implements the beahviour to decide the next room to explore
#   and the request to the action server move_base to move there.
#
def gotonewroom():
    # definition of global variables
    global finished, target, rooms, visited, counter,state
    # initialization of local variables
    alreadyvisited=1
    # while the random room is already one visited
    while alreadyvisited==1:
        # put already visited to zero
        alreadyvisited=0
        # generate another room number
        room= random.randint(0, 5)
        # debug print
        #print(room)
        # check if the new room id has already been visited
        for i in visited:
             if i==room:
                 alreadyvisited=1
    # when one id is of one room that has not been visited
    # set the request for the move_abse action server
    target.target_pose.pose.position.x=rooms[room][0]
    target.target_pose.pose.position.y=rooms[room][1]
    # send the request to the move_base action server
    client.send_goal(target)
    #print(target)
    #print("msg sent")
    # wait for the robot to reach the room
    client.wait_for_result()
    # add the room to the list of visited
    visited.append(room)
    # if all rooms are visited reset the visited list
    if len(visited)== 6:
        print("all rooms visited")
        visited.clear()
    # change to state 1: move around
    state=1

##
#	\brief 
#	\param : 
#	\return : 
# 	
#   
#
def movearound():
    global pub_,state, up 
    print("move")
    vel=Twist()
    vel.angular.z=0.6
    pub_.publish(vel)
    time.sleep(45)
    vel.angular.z=0
    pub_.publish(vel)
    state=0
    if up==False:
        state=3
    else:
        state=2

##
#	\brief 
#	\param : 
#	\return : 
# 	
#   
#
def moveup():
    global move_group, up, state
    print('moveup')
    joint_goal = move_group.get_current_joint_values()
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    move_group.go(joint_goal, wait=False)
    time.sleep(15)
    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    state=1
    up=True

##
#	\brief 
#	\param : 
#	\return : 
# 	
#   
#
def movedown():
    global move_group, up ,state
    print('movedown')
    joint_goal = move_group.get_current_joint_values()
    joint_goal[1] = pi/4
    joint_goal[2] = pi/4
    joint_goal[3] = -pi/2 + 0.05
    move_group.go(joint_goal, wait=False)
    time.sleep(15)
    print('sleep done')
    # Calling ``stop()`` ensures that there is no residual movement
    #move_group.stop()
    state=0
#	This function initializes all of the needed services
    up=False

##
#	\brief 
#	\param : 
#	\return : 
# 	
#   
#
def complete(rec):
    global pub_, state
    vel=Twist()
    vel.angular.z=0
    pub_.publish(vel)
    state=4

##
#	\brief 
#	\param : 
#	\return : 
# 	
#   
#
def gohome():
    target.target_pose.pose.position.x=0
    target.target_pose.pose.position.y=-1
    client.send_goal(target)
    print(target)
    print("msg sent")
    client.wait_for_result()

##
#	\brief 
#	\param : 
#	\return : 
# 	
#   
#
def completeupdate(received):
    global complete_hyp_checked, complete_hyp_to_check, complete_available
    found=0
    for i in complete_hyp_to_check:
        if i==received.data:
            found=1
    for i in complete_hyp_checked:
        if i==received.data:
            found=1
    if found==0:
        complete_hyp_to_check.append(int(received.data))
        complete_available=True
        client.cancel_goal()

##
#	\brief 
#	\param : 
#	\return : 
# 	
#   
#
def result():
    global complete_hyp_checked, complete_hyp_to_check,finished, complete_available, state
    win_id=oracle_solution_service()
    print(win_id)
    for i in complete_hyp_to_check:
        print("hypothesis %d" %i)
        complete_hyp_checked.append(i)
        if i==win_id.ID:
            respcall=result_service(i)
            print("GAME FINISHED")
            print( " %s with the %s in the %s" % (respcall.who , respcall.what ,respcall.where))
            finished=1
            return True
        complete_hyp_to_check.remove(i)
            
    state=0
    complete_available=False
         

##
#	\brief This function is called when the node is started
#	\param :
#	\return : None
# 	
#	This function initializes all of the needed services, then it calculates a new plan
#   until the robot is able to fuòòy complete one. 
def main():
    global pub_, active_, act_s, client, target, finished, move_group , state, oracle_solution_service, result_service
    rospy.init_node('finite_state_machine')
    # initialize the publisher to movebase goal 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    print("server up")
    #I initialize the publisher for the velocity
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # definition for the subsciber on the topic /completefound
    rospy.Subscriber('/complete_found', Bool, complete)
    # definition for the subsciber on the topic /completefound
    rospy.Subscriber('/complete', String, completeupdate)
    #definition for the Client  on the topic "/oracle_solution"
    oracle_solution_service = rospy.ServiceProxy("/oracle_solution", Oracle)
    rospy.wait_for_service('/oracle_solution')
    #definition for the Client  on the topic "/oracle_solution"
    result_service = rospy.ServiceProxy("/results", Results)
    rospy.wait_for_service('/results')

    target.target_pose.pose.orientation.w=1
    target.target_pose.header.frame_id="map"
    robot = moveit_commander.RobotCommander()
    random.seed= int(time.time() * 1000)
    group_name="arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    while finished==0:
        if complete_available:
             gohome()
             result()
        else:
            if state==0:
               gotonewroom()
            elif state==1:
                movearound()
            elif state==2:
                movedown()
            elif state==3:
                moveup()
        
    print ( 'SUCCESS')

if __name__ == '__main__':
    main()
