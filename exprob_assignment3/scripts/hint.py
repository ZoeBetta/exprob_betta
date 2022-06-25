#! /usr/bin/env python

## @package exprob_assignment3
#
#  \file hint.py
#  \brief This program elaborates the hints received
#
#  \author Zoe Betta
#  \version 1.0
#  \date 12/06/2022
#  \details
#  
#  Subscribes to: <BR>
#       /hint
#
#  Publishes to: <BR>
#	    /hypothesis
#		/complete
#
#  Services: <BR>
#       /hint
#		/checkcomplete
#		/result
#
#  Client Services: <BR>
#       armor_interface_srv
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    This node receives the hints published on the topic /hint. Then it is 
#    able to elaborate them: it check if the person, location or weapon is 
#    already present in the ontology. In case it is not in the ontology yet
#    it adds it. After that it checks if the hint has already been saved in
#    the ontology, if it is the first time the hint has been received it 
#    checks if the hypothesy that correspond to that hint ID is complete 
#    and not inconsistent. In that case it checks if the hypothesy has 
#    already been sent, if not it publish the hipothesy on the topic /hypothesis
#    that is of type Hypothesis.msg. 

import copy
import math
import sys
import time
import geometry_msgs.msg
import numpy as np
import rospy
from std_msgs.msg import String, Int32
from armor_msgs.msg import * 
from armor_msgs.srv import * 
from exprob_assignment3.srv import Marker, MarkerResponse
from exprob_assignment3.srv import HintElaboration
from exprob_assignment3.srv import Complete,CompleteResponse
from exprob_assignment3.srv import Results, ResultsResponse

#global variables
people=[]
weapons=[]
locations=[]
armor_service = None
pub= None
complcons=[]
retrievedId=[]
oracle_service = None

##
#	\brief This function implements the ros node
#	\param : None
#	\return : None
# 	
#   When the node is first initialized it loads all of the publishers, 
#   subscribers and servers. It also loads the ontology file needed to 
#   reason on the hints.
#	
def main():
  global  armor_service, pub, oracle_service
  rospy.init_node('hint')
  # definition of the Client for the Server on the topic armor_interface_srv
  armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
  print('waiting for armor server')
  # waits for the service to be correctly running
  rospy.wait_for_service('armor_interface_srv')
  # definition for the Server on the topic /hint
  service=rospy.Service('/hint', HintElaboration, hint)
  # definition of the Server on the topic /checkcomplete
  service=rospy.Service('/checkcomplete', Complete, checkcomplete)
  # definition of the Server on the topic /results
  service=rospy.Service('/results', Results, results)
  # definition of the publisher on the topic /complete
  pub = rospy.Publisher('/complete', String, queue_size=10)
  print('inizializzato tutto')
  # load the ontology from the ontology file
  load_ontology()
  rospy.spin() 


	
	
##
#	\brief This function implements the /results server
#	\param : req: it is an integer that contains the ID of an hypothesis
#	\return : resp: it is composed of three strings, one for each  field of the hypothesis
# 	
#   This server retrieves the field of a requested hypothesis,
#   identified by the ID that is sent as a request. 
#   The fields are retrieved in the armor server and sent back on the response.
#	
def results(req):
	# I initialize the response variable
    resp=ResultsResponse()
    # save the value retrieved in the field what in a temporary variable
    what=str(look_hypothesis(str(req.id), 'what')[0])
    # save the value retrieved in the field who in a temporary variable
    who=str(look_hypothesis(str(req.id), 'who')[0])
    # put the value for who in the response message
    resp.who=who
    # save the value retrieved in the field where in a temporary variable
    where=str(look_hypothesis(str(req.id), 'where')[0])
    # put the value for what in the response message
    resp.what=what
    # put the value for where in the response message
    resp.where=where
    return resp
	
##
#	\brief This function implements the /checkcomplete server
#	\param : req: a boolean to start checking if there is a complete hypothesis available
#	\return : resp: a boolean to state if there is a complete hypothesis or not
# 	
#   This server looks if in the armor server is present a complete hypothesis
#   if there is at least one complete hypothesis it returns True; False otherwise.
#		
def checkcomplete(req):
	# check if the hypothesis is complete and consistent
    print('request to check complete')
    send=check_complete_consistent()
    # I check the value returned from the function check_complete_consistent
    # if tìit is one I have a complete hypothesis
    if send==1:
	    return CompleteResponse(True)
	# else I don't have any complete and consistent hypothesis.
    else : 
	    return CompleteResponse(False)
	
	
##
#	\brief It checks if an hypothesis is complete and not inconsistent
#	\param None
#	\return : returns 1 if the hypothesis is complete and not inconsistent
#    returns 0 if it is either incomplete or inconsistent.
# 	
#	This function calls the armor server twice, the first time it retrieves all
#   the complete hypothesis and it checks if there is at least one hypothesis that is
#   complete and not inconsistent
def check_complete_consistent():
    try:
        completed=[]
        inconsistent=[]
        temp=[]
        # set the request for the armor server check all the completed hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        completed=res_final
        # set the request for the armor server check all the inconsistent hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        inconsistent=res_final
        # if the hypothesis is completed AND inconsistent return 1
        # for every element in the array completed
        for i in range(len(completed)):
            dontdo=0
            # for every element in the list inconsistent
            for j in range(len(inconsistent)):
				# if the two elements are the same I have a complete and inconsistent hypothesis
                if completed[i]==inconsistent[j]:
					# I have a flag to say I can't send this element of complete
                    dontdo=1
            # if the element of complete is different from all elements of inconsistent
            if dontdo==0:
			    # Save that element in a temporary list
                temp.append(completed[i])
        # If at leat one element is in the list of complete and consistent
        if len(temp)>0:
            complcons=temp
            # publish all elements on the topic /complete
            for i in range(len(temp)):
                pub.publish(temp[i])
            return 1
        # else the list is empty return 0.
        else :
            return 0
    except rospy.ServiceException as e:
        print(e)
	
	
##
#	\brief This function implements the server /hint
#	\param req: the data that needs to be elaborated
#	\return : ret a boolean to signal if the hint has been elaborated correctly
# 	
#	This function elaboratesthe hints received. It checks if the hint received has
#   been received correctly or there are some missing fields. If the hint is 
#   correct it then checks if it has already been saved in the ontology and 
#   in case it is necessary it provides to save it
def hint(req):
    already_done=0
    print('message received')
    hint_received=[]
    # check if ID is correct: not empty and not -1
    if str(req.ID)=="" or req.ID==-1:
	    print('id wrong')
	    return False
	# check if the key is correct: not empty, not -1 and not a value different from the allowed ones
    if str(req.key)=="" or str(req.key)=='0' or str(req.key)=='-1' or (str(req.key)!='who' and str(req.key)!='what' and str(req.key)!='where'):
	    print('key wrong')
	    return False
	# check if the value is correct: not empty, 0, -1 
    if str(req.value)=="" or str(req.value)=='0' or str(req.value)=='-1':
	    print('value wrong')
	    return False
	# only if the hint is correct I reach this point
	# I save the hint on a list casting all the elements to be strings
    hint_received.append(str(req.ID))
    hint_received.append(str(req.key))
    hint_received.append(str(req.value))
    # uncomment if you want to print the received message
    print(hint_received[0])
    print(hint_received[1])
    print(hint_received[2])
    # check if the hint was already received and so the fields are already saved in the ontology
    find=check_if_received_before(hint_received)
    # if it was never received before
    if find==0:
        # add to the ontology
        add_instance(hint_received[2],hint_received[1])
    # check if the hint is already in the ontology	    
    check_in_ontology(hint_received[0], hint_received[1], hint_received[2])
    return True


##
#	\brief This function loads the ontology
#	\param : None 
#	\return : None
# 	
#	It loads the ontology from a file called cluedo_ontology in a specific
#   path. Be careful if you want to change the path of the file it needs
#   to be done here. The file is loaded by the proper call to the armor server.
def load_ontology():
    try:
        # set the request for the armor server to load ontology
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'LOAD'
        req.primary_command_spec= 'FILE'
        req.secondary_command_spec= ''
        # IT IS NECESSARY TO CHANGE THE FOLLOWING LINE IF THE FILE IS NOT THERE
        # CHANGE THE FIRST ARGUMENT WITH THE CORRECT PATH
        req.args= ['/root/ros_ws/src/exprob_assignment2/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        # send the message on the server
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)
 
##
#	\brief This function adds an instance in the ontology
#	\param name : it is a string representing the name of the instance
#   \param class_type: it is a string representing the type of the class,
#                       it can be who, what or where
#	\return : None
# 	
#	 This function adds one entity to the ontology. First it checks which
#    class the entity belongs to and then it adds it to the ontology by
#    making the proper request to the armor server. After that it reasons and
#    specify that the new entity is not equal to any previous entities of
#    the same class      
def add_instance(name, class_type):
    try:
        # from the class type (who, what,where) find the class (PERSON,LOCATION,WEAPON)
        class_id=find_type(class_type)
        # set the request for the armor server to add an instance of a class
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [name, class_id]
        # send the request
        msg = armor_service(req)
        res=msg.armor_response
        # Use the reasoner of the ontlogy
        reason()
        # Specify the element of the class are different, disjoint
        disjoint(class_id)
        # Use the reasoner of the ontlogy
        reason()
    except rospy.ServiceException as e:
        print(e)

##
#	\brief this function returns the class name corresponding to the given type
#	\param class_type : it is a string, it can be who, what or where 
#	\return : The class name
# 	
#	This function checks the type of the class given as input parameter and
#   returns the string with the corresponding class name.
def find_type(class_type):
    if class_type=='who':
        return 'PERSON'
    if class_type== 'what':
        return 'WEAPON'
    if class_type=='where':
        return 'LOCATION' 

##
#	\brief this function updates the ontology
#	\param : None
#	\return : None
# 	
#	 This function calls the armor server in order to run the reasoner
#    and make implicit knowledge explicit       
def reason():
    try:
        # set the request for the armor server to use the reasoner
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        # send the request
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)		
 
##
#	\brief This function specifies that all elements of a class are different
#	\param class : of type string it is the class of which element I want to make disjoint
#	\return : None
# 	
#	This function calls the armor server and by sending specific commands it
#   specifies that all entities inside the class passed as parameter are 
#   disjoint and different   
def disjoint(class_id):
    try:
        # set the request for the armor server to use the reasoner
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [class_id]
        # send the request
        msg = armor_service(req)		 
    except rospy.ServiceException as e:
        print(e)        

##
#	\brief This function cleans the query returned from the ontology
#	\param query: the list of strings that needs to be cleaned
#	\return : query, the cleaned query
# 	
#	This function, for each element of the list passed as input it splits
#   the strings at the char '#' and takes only what is after. Then it removes
#   the last element of the remaing string. The received query is of type 
#   <http://www.emarolab.it/cluedo-ontology#rope> and in this example
#   we want to extract rope
def clean_queries(query):
    # for every element of the list received as input 
    for i in range(len(query)):
        temp=query[i]
        # split at the character '#'
        temp=temp.split('#')
        # save the lenght of the list returned after the split
        index=len(temp)
        # take only the last element ( lenght -1 ) 
        temp=temp[index-1]
        # saves it in the query, overwriting the one received and 
        # eliminating the last character
        query[i]=temp[:-1]
    return query

##
#	\brief This function checks if the entity has already been received
#	\param data: it is a list of strings I want to check if it has been received before
#	\return : find an integer that is 1 if the entity was already received
#             at least once and 0 if the entity was not received before.
# 	
#	 In this function, depending on the type of string that has been received, if 
#    a person, a location or a weapon, it checks in the corresponding global list 
#    if already present. If that is not the case it then adds it at the end.   
def check_if_received_before(data):
    global people, weapons, locations
    find=0
    i=0
    j=0
    k=0
    # if the data received is of class_type 'who'
    if data[1]=='who':
        # checks every element of the array people to see if the data received is already saved there
        for i in range(len(people)):
            if people[i]==data[2]:
                find=1;
        # if it is not there
        if find==0:
            # I add it to the global list
            people.append(data[2])
    # if the data received is of class_type 'what'
    if data[1]=='what':
        # checks every element of the array weapons to see if the data received is already saved there
        for j in range(len(weapons)):
            if weapons[j]==data[2]:
                find=1;
        # if it is not there
        if find==0:
            # I add it to the global list
            weapons.append(data[2])
    # if the data received is of class_type 'where'
    if data[1]=='where':
        # checks every element of the array locations to see if the data received is already saved there
        for k in range(len(locations)):
            if locations[k]==data[2]:
                find=1;
        # if it is not there
        if find==0:
            # I add it to the global list
            locations.append(data[2])
    return find            

##
#	\brief This function adds an hypothesis to the ontology
#	\param ID : of type string it is the ID of the hypothesis I want to add to
#	\param class_type : string representing the type of information I want to add
#   \param name : of type string it is the name of the information I want to add
#	\return : None
# 	
#	By calling the armor server with the proper commands I add an entity to
#   a given hypothesis. 
def add_hypothesis(ID,class_type,name):
    try:
        # set the request for the armor server to add an entity to an hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID,name]
        # send the request
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)	

##
#	\brief It retrieves from an hypothesis a field
#	\param ID : of string type it is the hypothesis I want to check 
#   \param class_type : the property of the hypothesis that I want to retrieve
#	\return : res_final a string with the name of the entity I retrieved
# 	
#	This funciton calls the armor server to see in a given hypothesis identified
#   by its ID one field, identified by the class_type.
def look_hypothesis(ID,class_type):
    try:
        # set the request for the armor server to check one field (identified by class_type)
        # of an hypothesis (identified by an ID)
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID]
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        #print(res)
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        return res_final
    except rospy.ServiceException as e:
        print(e)   

##
#	\brief It checks if the hint received is already saved in the hypothesis
#	\param ID: of type string, the ID of the hypothesis I want to check 
#	\param class_type: of type string, the type of instance I want to check
#       if already present
#	\param name : of type string, the name of the entity I want to check
#	\return : None
# 	
#	This function checks if a hint ( composed of an ID, class_type and name) 
#   is already present in an hypothesis. If it is not present it adds it 
#   to the ontology.
def check_in_ontology(ID,class_type,name):
    try:
        namef=[]
        # it retrieves from the hypothesis ID the field identified by class_type
        res_final=look_hypothesis(ID, class_type)
        # add the name to namef
        namef.append(name)
        # if the name retrieved is different from the name received I add the hypothesis
        # it adds it even if the retrieved field is empty
        if res_final != namef:
            add_hypothesis(ID,class_type,name)
            # use the reasoner for the ontology
            reason()
    except rospy.ServiceException as e:
        print(e)      


if __name__ == '__main__':
  main()        
        
   
