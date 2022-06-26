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
    print("go to new room")
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
#	\brief function to rotate around
#	\param : None
#	\return : None
# 	
#   This function implements the behaviour to rotate around the z-axis of
#   the robot in order to look around the room and retrieve hints
#
def movearound():
    #definition of global variables
    global pub_,state, up 
    print("move")
    # definition of local variables
    vel=Twist()
    # setting the desired velocity
    vel.angular.z=0.6
    # publish the velocity on the topic /cmd_vel
    pub_.publish(vel)
    # sleep for a time sufficent to do at least one full turn
    time.sleep(45)
    # setting the velocity to zero
    vel.angular.z=0
    # publish the velocity on the topic /cmd_vel
    pub_.publish(vel)
    # if the arm is down
    if up==False:
        # set next state as 3 : moveup
        state=3
    # if the arm is up
    else:
        # set the next state as 2 : movedown
        state=2

##
#	\brief function to move the arm in the up position
#	\param : None
#	\return : None
# 	
#   This function implements the call to the moveit server to move the arm
#   to the desired upward position
#
def moveup():
    # definition of global variables
    global move_group, up, state
    print('moveup')
    # retrieve the current position of the robot
    joint_goal = move_group.get_current_joint_values()
    # set the goal joint positions
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    # send the goal to the moveit server
    move_group.go(joint_goal, wait=False)
    # wait for the arm to reach the position
    time.sleep(15)
    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    # setting the next state to 1 : move around
    state=1
    # the arm is upwards
    up=True

##
#	\brief function to move the arm in the down position
#	\param : None
#	\return : None
# 	
#   This function implements the call to the moveit server to move the arm
#   to the desired downward position
#
def movedown():
    #definition of global variables
    global move_group, up ,state
    print('movedown')
    # retrieve the current position of the robot
    joint_goal = move_group.get_current_joint_values()
    # set the goal joint position
    joint_goal[1] = pi/4
    joint_goal[2] = pi/4
    joint_goal[3] = -pi/2 + 0.05
    # send the goal to the moveit server
    move_group.go(joint_goal, wait=False)
    # wait for the arm to reach the position
    time.sleep(15)
    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    # setting the next state to 0 : gotonewroom
    state=0
    # the arm is downward
    up=False

##
#	\brief this function implements the request to move home 
#	\param : None
#	\return : None
# 	
#   This function implements the action server request to move to the home position
#
def gohome():
    print("go home")
    # initialization of the target position
    target.target_pose.pose.position.x=0
    target.target_pose.pose.position.y=-1
    # sending the requesto to the move_base action server
    client.send_goal(target)
    #print(target)
    #print("msg sent")
    # wait for the robot to reach the goal position
    client.wait_for_result()

##
#	\brief callback on the topic /complete
#	\param : received
#	\return : None
# 	
#   This function receives all the available ID corresponding to complete and
#   consistent hypothesis. When an ID is received it is checked if that ID has
#   already been received or checked. If that is not the case all the move_base goal
#   are preempted and the state is changed in order for the robot to go home.
#
def completeupdate(received):
    # definition of global variables
    global complete_hyp_checked, complete_hyp_to_check, complete_available,state
    print("complete update")
    # definition of local variables
    found=0
    # check if the ID has already been received
    for i in complete_hyp_to_check:
        if i==received.data:
            found=1
    # check if the ID has already been compared to the winning ID
    for i in complete_hyp_checked:
        if i==received.data:
            found=1
    # if it is the first time I see the ID
    if found==0:
        # add the ID to the list of available IDs
        complete_hyp_to_check.append(int(received.data))
        # set a global variable to true to change the stateof execution
        complete_available=True
        # change the state to 4
        state=4
        # if the state is not 4, it is the first time this function is called
        if state!=4:
            # cancel the goal
            client.cancel_goal()

##
#	\brief this function check if the hypothesis is the winning one
#	\param : None
#	\return : 
# 	
#   Thi function calls the server to retrieve the winning id and compares it to the
#   list of id of complete and consistent hypothesis. If one ID is  the correct
#   one the execution stops and the program finishes. If no id matches the 
#   winning ID the program goes back to normal execution: state 2, move the arm down.
#
def result():
    # definition of global variables
    global complete_hyp_checked, complete_hyp_to_check,finished, complete_available, state
    print("result")
    # call the server to retrieve the winning id
    win_id=oracle_solution_service()
    #print(win_id)
    # for every element in the list of hypothesis complete and consistent
    for i in complete_hyp_to_check:
        print("hypothesis %d" %i)
        # add the id to the list of checked ids
        complete_hyp_checked.append(i)
        # if the id is the same as the winning one
        if i==win_id.ID:
            # call the server /results to retrieve the fields of the hypothesis
            respcall=result_service(i)
            print("GAME FINISHED")
            print( " %s with the %s in the %s" % (respcall.who , respcall.what ,respcall.where))
            # set finished to true to interrupt the execution of this program
            finished=1
            return True
        # remove the id from the list of ids that needs to be checked
        complete_hyp_to_check.remove(i)
    # if no id matches the winning id return to state 2: move down
    state=2
    # set this variable to false to exit this execution
    complete_available=False
         

##
#	\brief This function is called when the node is started
#	\param :
#	\return : None
# 	
#	This function initializes all of the needed services and implements
#   the while cycle that calls all of the functions
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
    rospy.Subscriber('/complete', String, completeupdate)
    #definition for the Client  on the topic "/oracle_solution"
    oracle_solution_service = rospy.ServiceProxy("/oracle_solution", Oracle)
    rospy.wait_for_service('/oracle_solution')
    #definition for the Client  on the topic "/oracle_solution"
    result_service = rospy.ServiceProxy("/results", Results)
    rospy.wait_for_service('/results')

    # initialize some fields for the move_base action goal
    target.target_pose.pose.orientation.w=1
    target.target_pose.header.frame_id="map"
    # initialize a random seed to ensure true randomness in the choice of the rooms
    random.seed= int(time.time() * 1000)
    # initialize the moveit commander
    robot = moveit_commander.RobotCommander()
    group_name="arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    # while cycle that implements the finite state machine
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
