# Brief Introduction
This project implements the game of Cluedo played by a robot that has to explore an unknown environment searching for hints. The robot should move in the environment, that has six rooms looking for hints that are placed in unknown locations. When the robot is ready to make an hypothesis it should return to the center of the room and ask the oracle if the hypothesis is correct. The logic behind the behaviour of the robot is based on a finite state machine.  

# Software Architectture
![diagram of the software architecture](https://github.com/ZoeBetta/exprob_betta/blob/main/exprob_assignment3/documentation/images/architecture.jpg)
In this diagram is represented the structure of the software architecture. I wrote three nodes: aruco.py, fsm.py and hint.py. The simulation node was provided by Professor Carmine Recchiuto in the pachage: https://github.com/CarmineD8/exp_assignment3/. The other packages can be manually installed and are freely available on the web.  
The node aruco.py has the objective of receiving the data from the aruco package, for this reason it subscribes to the topic /marker_publisher/detected_id that contains an int32 representing the id of the detected aruco marker. This node also implements two controls: it controls if the marker has been detected before and it controls if the id is a valid one. With the first control the goal was to avoid as much as possible computational burden, in fact by manually listening to the /marker_publisher/detected_id it is possible to notice that the same marker is detected usually more than once and very rapidly, with that check it is avoided the repetitive request on both the /oracle_hint server and the /hint server. It was also noted that sometimes the package would recognize markers with an id that was not one of the marker in the scene. This would cause the node simulation.cpp to crash since it would try to access a memory that was not allocated. In order to avoid chaging that file I control the validity of the id before sending it to the /oracle_hint server.  

# Installation

# Running Code

# How the code runs

# Working Hypothesis

## System's Features

## System's Limitations

## Possible Techinical Improvements

# Authors 
Zoe Betta  
s5063114@studenti.unige.it  
zoe.betta@gmail.com
