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

def gotonewroom():
    global finished, target, rooms, visited, counter,state
    alreadyvisited=1
    while alreadyvisited==1:
        alreadyvisited=0
        room= random.randint(0, 5)
        print(room)
        for i in visited:
             if i==room:
                 alreadyvisited=1
    target.target_pose.pose.position.x=rooms[room][0]
    target.target_pose.pose.position.y=rooms[room][1]
    client.send_goal(target)
    print(target)
    print("msg sent")
    client.wait_for_result()
    visited.append(room)
    counter=counter+1
    if len(visited)== 6:
        print("all rooms visited")
        visited.clear()
    state=1

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
    up=False

def complete(rec):
    global pub_, state
    vel=Twist()
    vel.angular.z=0
    pub_.publish(vel)
    state=4

def gohome():
    target.target_pose.pose.position.x=0
    target.target_pose.pose.position.y=-1
    client.send_goal(target)
    print(target)
    print("msg sent")
    client.wait_for_result()

def completeupdate(received):
    global complete_hyp_checked, complete_hyp_to_check, complete_available
    found=0
    for i in complete_hyp_checked:
        if i==received:
            found=1
    for i in complete_hyp_to_check:
        if i==received:
            found=1
    if found==0:
        complete_hyp_to_check.append(received)
        complete_available=True

def result():
    global complete_hyp_checked, complete_hyp_to_check,finished, complete_available
    win_id=oracle_solution_service()
    print(win_id)
    complete_hyp_checked.append(complete_hyp_to_check[0])
    print(complete_hyp_to_check[0])
    if int(str(complete_hyp_to_check[0]))==win_id:
        respcall=result_service(int(str(complete_hyp_to_check[0])))
        print("GAME FINISHED")
        print( " %s with the %s in the %s" % (respcall.who , respcall.what ,respcall.where))
        finished=1
    else:
        complete_hyp_to_check.pop(0)
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
