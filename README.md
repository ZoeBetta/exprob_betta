# Brief Introduction
This project implements the game of Cluedo played by a robot that has to explore an unknown environment searching for hints. The robot should move in the environment, that has six rooms looking for hints that are placed in unknown locations. When the robot is ready to make an hypothesis it should return to the center of the room and ask the oracle if the hypothesis is correct. The logic behind the behaviour of the robot is based on a finite state machine.  

# Software Architectture
![diagram of the software architecture](https://github.com/ZoeBetta/exprob_betta/blob/main/exprob_assignment3/documentation/images/architecture.jpg)
In this diagram is represented the structure of the software architecture. I wrote three nodes: aruco.py, fsm.py and hint.py. The simulation node was provided by Professor Carmine Recchiuto in the pachage: https://github.com/CarmineD8/exp_assignment3/. The other packages can be manually installed and are freely available on the web.  
Communication in this architecture has been achieved with either ros publisher and subscriber: when there was no need for the synchronization between nodes or the server client structure when the client should wait for the answer from the server to continue the execution.  
The node aruco.py has the objective of receiving the data from the aruco package, for this reason it subscribes to the topic /marker_publisher/detected_id that contains an int32 representing the id of the detected aruco marker. This node also implements two controls: it controls if the marker has been detected before and it controls if the id is a valid one. With the first control the goal was to avoid as much as possible computational burden, in fact by manually listening to the /marker_publisher/detected_id it is possible to notice that the same marker is detected usually more than once and very rapidly, with that check it is avoided the repetitive request on both the /oracle_hint server and the /hint server. It was also noted that sometimes the package would recognize markers with an id that was not one of the marker in the scene. This would cause the node simulation.cpp to crash since it would try to access a memory that was not allocated. In order to avoid chaging that file I control the validity of the id before sending it to the /oracle_hint server. Whenever a new hint is available this node sends it to the /hint server to have it elaborated by the ontology, waits for the response and if the hint is well-formed, no missing or wrong fields, it then checks if a complete and consistent hypothesis can be made.     
The node fsm.py is responsible to control the execution of the software. It calls the needed server to move the robot in the environment, waits for the correct execution of said servers and listens to the topic /complete. When data are available on the topic /complete the node stops the normal execution of the program, moves the robot to the home position and checks if one of the id received on the topic /complete are the winning id. If that it retrieves, using the server /result, the fields of the correct hypothesis, it prints the solution on the screen and it stops the execution. If the winning id has not been found in the complete and consisten hypothesis the fsm resumes the previous execution and moves the robot to keep looking for hints.  
The node hint.py as previously stated implements the three servers /hint, /checkcomplete, /results. All of these servers are based on the knowledge in the ontology. The server /hint checks if the hint is correctly formed, if all fields are filled and if what is written in the fields is meaningful. In case everything is correct the hint is saved in the ontology. The server /checkcomplete retrieves from the ontology all of the hypothesis, identified by a unique ID, that are complete and all the hypothesis that are not consistent. It then checks if there is at least one hypothesis that is in the complete list but not in the inconsistent list. All of the hypothesis that are both complete and consistent are then published on the topic /complete, if at least one hypothesis has been retrieved the server returns true, false otherwise. The server /results receives an ID and has to retrieve from the ontology all of the fields of that specific hypothesis; the fields are then returned as response of the server.  


# Installation
In order to run this software several additional packages are needed:
* aruco_ros : available for download at the following link https://github.com/CarmineD8/aruco_ros.git
* erl2 : available for download at the following link https://github.com/CarmineD8/erl2.git
* moveit : can be installed by typing apt-get ros-noetic-moveit and apt-get ros-noetic-moveit-resources-prbt-moveit-config
* move_base : available for download at the following link https://github.com/ros-planning/navigation.git
* armor : available for download at the following link https://github.com/EmaroLab/armor. To install follow the instruction in the README file. In particulr it is important to run the following command ./gradlew deployApp in the armor folder.

An alternative to manually install all of this packages is download the docker image at the following link: https://hub.docker.com/r/carms84/exproblab  
In order to properly run the simulation copy the model folder inside the ~/.gazebo/models folder.
To build the workspace run catkin_make
# Running Code
To run the code it is sufficient to run in a terminal
roslaunch exprob_assignment3 simulation.launch  
When the robot finds the correct id it is necessary to stop the program by pressing CTR+C in the terminal.

# How the code runs
![state machine](https://github.com/ZoeBetta/exprob_betta/blob/main/exprob_assignment3/documentation/images/state_diagram.jpg)

![goal reached](https://github.com/ZoeBetta/exprob_betta/blob/main/exprob_assignment3/documentation/images/finished.JPG)

![rviz interface](https://github.com/ZoeBetta/exprob_betta/blob/main/exprob_assignment3/documentation/images/rviz%20interface.JPG)



https://user-images.githubusercontent.com/77151364/175827458-80096719-887a-4c2d-a867-30196b2c10ad.mp4


# Working Hypothesis

## System's Features

## System's Limitations

## Possible Techinical Improvements

# Authors 
Zoe Betta  
s5063114@studenti.unige.it  
zoe.betta@gmail.com
