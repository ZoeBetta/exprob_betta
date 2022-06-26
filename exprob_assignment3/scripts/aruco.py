#! /usr/bin/env python

## @package exprob_assignment3
#
#  \file aruco.py
#  \brief this file receives the hint from the aruco marker detector
#
#  \author Zoe Betta
#  \version 1.0
#  \date 12/06/2022
#  \details
#  
#  Subscribes to: <BR>
#  /marker_publisher/detected_id	 
#
#  Publishes to: <BR>
#	None
#
#  Services: <BR>
#    None
#
#  Action Services: <BR>
#  None
#
#  Client Services: <BR>
#  /oracle_hint
#  /hint
#  /checkcomplete
#    
#
#  Description: <BR>
#  This program listens for the ID retrieved from the aruco marker detector,
#  it then calls the server that given the aruco marker returns the hint,
#  sends the hint to the hint elaboration server and checks if a complete
#  and consistent hypothesis is available


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
#	\brief this function elaborates the marker retrieved
#	\param : idtocheck the id received from the publisher
#	\return : None
# 	
#   This function implements the callback on the topic /marker_publisher/detected_id
#   that retrieves the aruco marker detected. Once the marker is received 
#   it is checked if the same id has already been found, in this case the function does nothing.
#   If the id has never been received before it is checked if the ID is in the
#   correct range. If it is in the correct range it is sent a request to the
#   server to retrieve the hint. When the hint is retrieved it is sent
#   a request to the server that elaborates the hints, if the server returns True 
#   (the hint was well-formed) then we request to another server if there is at 
#   least one complete and consistent hypothesis. 
#
def newId (idtocheck):
    # global variables
    global oracle_service, pub_complete
    # variables initialization
    req_elab=HintElaborationRequest()
    found=0
    req=Marker()
    # check if the id has already been found
    for x in retrievedId:
        if (x==idtocheck.data):
            found=1
    # if it was not found before
    if found==0:
        # add the id to the list of already seen ids
        retrievedId.append(idtocheck.data)
        # check if the id is in the correct range
        if (idtocheck.data>10) and (idtocheck.data<41):
            # initialize the request to the server /oracle_hint
            req=idtocheck.data
            # call the server
            hint=oracle_service(req)
            # debug prints
            #print(hint)
            #print(hint.oracle_hint.ID)
            #print(hint.oracle_hint.key)
            #print(hint.oracle_hint.value)
            # initialize the request to the server /hint
            req_elab.ID=hint.oracle_hint.ID
            req_elab.key=hint.oracle_hint.key
            req_elab.value=hint.oracle_hint.value
            # request to the server /hint
            res=hint_elab_service(req_elab)
            #print(res.ret)
            # if the response is true check if complete and consistent
            if res.ret:
                # call to the server /checkcomplete
                res_compl=complete_service(True)
                print(res_compl)     
        else:
            print("id out of range")

##
#	\brief This function is called when the node is started
#	\param :
#	\return : None
# 	
#	This function initializes all of the needed services, publishers and subscribers
def main():
    global pub_complete, active_, act_s, oracle_service, hint_elab_service, complete_service
    rospy.init_node('aruco')
    #definition for the Client  on the topic /oracle_hint
    oracle_service = rospy.ServiceProxy('oracle_hint', Marker)
    rospy.wait_for_service('/oracle_hint')
    # definition for the subsciber on the topic /marker_publisher/detected_id
    rospy.Subscriber("/marker_publisher/detected_id", Int32, newId)
    #definition for the Client  on the topic /hint
    hint_elab_service = rospy.ServiceProxy('/hint', HintElaboration)
    rospy.wait_for_service('/hint')
    #definition for the Client  on the topic /checkcomplete
    complete_service = rospy.ServiceProxy('/checkcomplete', Complete)
    rospy.wait_for_service('/checkcomplete')
    print("server up")
    rospy.spin() 

if __name__ == '__main__':
    main()
