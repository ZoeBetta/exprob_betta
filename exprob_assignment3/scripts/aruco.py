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
from exprob_assignment3.srv import Marker, MarkerResponse
from exprob_assignment3.srv import HintElaborationRequest, HintElaboration
from std_msgs.msg import String, Int32, Bool
from exprob_assignment3.srv import CompleteRequest, Complete, CompleteResponse
import time
import random
#global variables
retrievedId=[]
oracle_service = None
hint_elab_service=None
complete_service=None
pub_complete=None

##
#	\brief 
#	\param : 
#	\return : 
# 	
#   
#
def newId (idtocheck):
    global oracle_service, pub_complete
    req_elab=HintElaborationRequest()
    found=0
    req=Marker()
    for x in retrievedId:
        if (x==idtocheck.data):
            found=1
    if found==0:
        retrievedId.append(idtocheck.data)
        if (idtocheck.data>10) and (idtocheck.data<41):
            req=idtocheck.data
            hint=oracle_service(req)
            #print(hint)
            print(hint.oracle_hint.ID)
            #print(hint.oracle_hint.key)
            #print(hint.oracle_hint.value)
            req_elab.ID=hint.oracle_hint.ID
            req_elab.key=hint.oracle_hint.key
            req_elab.value=hint.oracle_hint.value
            res=hint_elab_service(req_elab)
            print(res.ret)
            # if the response is true check if complete and consistent
            if res.ret:
                res_compl=complete_service(True)
                if res_compl.ret:
                    pub_complete.publish(True)
                print(res_compl)     
        else:
            print("id out of range")

##
#	\brief This function is called when the node is started
#	\param :
#	\return : None
# 	
#	This function initializes all of the needed services, then it calculates a new plan
#   until the robot is able to fuòòy complete one. 
def main():
    global pub_complete, active_, act_s, oracle_service, hint_elab_service, complete_service
    rospy.init_node('aruco')
    #definition for the Client  on the topic /oracle_hint
    oracle_service = rospy.ServiceProxy('oracle_hint', Marker)
    print("wait for oracle")
    rospy.wait_for_service('/oracle_hint')
    # definition for the subsciber on the topic /marker_publisher/detected_id
    rospy.Subscriber("/marker_publisher/detected_id", Int32, newId)
    #definition for the Client  on the topic /hint
    hint_elab_service = rospy.ServiceProxy('/hint', HintElaboration)
    rospy.wait_for_service('/hint')
    complete_service = rospy.ServiceProxy('/checkcomplete', Complete)
    rospy.wait_for_service('/checkcomplete')
    #I initialize the publisher for the velocity
    pub_complete= rospy.Publisher('/complete_found', Bool, queue_size=1)
    print("server up")
    rospy.spin() 
    print ( 'SUCCESS')

if __name__ == '__main__':
    main()
