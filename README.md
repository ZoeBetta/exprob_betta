# Brief Introduction
This project implements the game of Cluedo played by a robot that has to explore an unknown environment searching for hints. The robot should move in the environment, that has six rooms looking for hints that are placed in unknown locations. When the robot is ready to make an hypothesis it should return to the center of the room and ask the oracle if the hypothesis is correct. The logic behind the behaviour of the robot is based on a finite state machine.  

# Software Architectture
![diagram of the software architecture](https://github.com/ZoeBetta/exprob_betta/blob/main/exprob_assignment3/documentation/images/architecture.jpg)
In this diagram is represented the structure of the software architecture. I wrote three nodes: aruco.py, fsm.py and hint.py. The simulation node was provided by Professor Carmine Recchiuto in the package: https://github.com/CarmineD8/exp_assignment3/. The other packages can be manually installed and are freely available on the web, I will explain in the paraghraph Installation how to download them.  
Communication in this architecture has been achieved with either ros publisher and subscriber (when there was no need for the synchronization between nodes) or the server client structure (when the client should wait for the answer from the server to continue the execution).  
The node aruco.py has the objective of receiving the data from the aruco package, for this reason it subscribes to the topic /marker_publisher/detected_id that contains an int32 representing the id of the detected aruco marker. This node also implements two controls: if the marker has been detected before and if the id is a valid one. With the first control the goal was to avoid as much as possible computational burden, in fact by manually listening to the /marker_publisher/detected_id it is possible to notice that the same marker is detected usually more than once and very rapidly, with that check it is avoided the repetitive request on both the /oracle_hint server and the /hint server. It was also noted that sometimes the package would recognize markers with an id that was not one of the marker in the scene. This would cause the node simulation.cpp to crash since it would try to access a memory that was not allocated. In order to avoid changing that file I control the validity of the id before sending it to the /oracle_hint server. Whenever a new hint is available this node sends it to the /hint server to have it elaborated by the ontology, waits for the response and if the hint is well-formed, no missing or wrong fields,then it checks if a complete and consistent hypothesis can be made.     
The node fsm.py is responsible to control the execution of the software. It calls the needed server to move the robot in the environment, waits for the correct execution of said servers and listens to the topic /complete. When data are available on the topic /complete the node stops the normal execution of the program, moves the robot to the home position and checks if one of the id received on the topic /complete are the winning id. If that is the case it retrieves, using the server /result, the fields of the correct hypothesis, it prints the solution on the screen and it stops the execution. If the winning id has not been found in the complete and consisten hypothesis the fsm resumes the previous execution and moves the robot to keep looking for hints.  
The node hint.py as previously stated implements the three servers /hint, /checkcomplete, /results. All of these servers are based on the knowledge in the ontology. The server /hint checks if the hint is correctly formed, if all fields are filled and if what is written in the fields is meaningful. In case everything is correct the hint is saved in the ontology. The server /checkcomplete retrieves from the ontology all of the hypothesis, identified by a unique ID, that are complete and all the hypothesis that are not consistent. It then checks if there is at least one hypothesis that is in the complete list but not in the inconsistent list. All of the hypothesis that are both complete and consistent are then published on the topic /complete, if at least one hypothesis has been retrieved the server returns true, false otherwise. The server /results receives an ID and has to retrieve from the ontology all of the fields of that specific hypothesis; the fields are then returned as response of the server.  
The aruco_ros package is needed to recognize, from the camera stream, the aruco markers, to identify them and to publish their id.
The moveit package is used to control the arm of the robot, in particular the motion of the arm that are used to explore the environment and retrieve hints.  
The move_base package, that relies on the SLAM algorithm is used to move the robot in an unknown environment by having it create a map while navigating and search in that map using the Global Planner for both the local and the global path.  
The armor package is neeeded to update the ontology and retrieve information that are saved there.  
The node simulation.cpp implements the oracle, it gives the hint when a marker is detected and it gives the winning id when called.

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

The code is based on this state machine. When the software starts the robot moves the arm to the downward position since I noticed that if moving whith the arm down the robot would get more markers. After moving the arm down, if no complete hypothesis has been received the robot will move to a new room. In the new room the robot will rotate around his z-axis to try to recognize as many hints as possibile. It will rotate once with the arm down, move the arm up and rotate again.

https://user-images.githubusercontent.com/77151364/175827458-80096719-887a-4c2d-a867-30196b2c10ad.mp4

In this video we can observe the behaviour described before when the robot reaches a room.  
If during any of these actions the robot receives the ID of a complete and consistent hypothesis it will move to the Home position and from there call the server /oracle_solution to compare the winning id to the complete and consistent hypothesisis found. If the winning id matches one of them the goal has been reached and the program stops. If instead no complete hypothesis is the correct one the robot starts  again from the state move arm down.  
When the execution is completed on the terminal there will be this message, informing that the solution has been found and what is it.
![goal reached](https://github.com/ZoeBetta/exprob_betta/blob/main/exprob_assignment3/documentation/images/finished.JPG)

During the execution of the program two graphical interfaces will run: Gazebo and Rviz.  
The following picture shows the Rviz interface:  

![rviz interface](https://github.com/ZoeBetta/exprob_betta/blob/main/exprob_assignment3/documentation/images/rviz%20interface.JPG)

We can notice that on the left we have both the planning interface of moveit and the camera stream to see what the camera is recording. On the right instead we have the robot in the environment. We can see the map that is being built with the move_base algorithm that is based on SLAM and the global path the robot is plannign to follow to reach the desired position.

The hints are formed of three fields: who, what, where.  
The possible values that the person can have are: 
* missScarlett
* colonelMustard
* mrsWhite
* mrGreen
* mrsPeacock
* profPlum

The possible values that the weapon can have are:
* candlestick
* dagger
* leadPipe
* revolver
* rope
* spanner

The possible values that the location can have are:
* conservatory
* lounge
* kitchen
* library
* hall
* study
* bathroom
* diningRoom
* billiardRoom

# Working Hypothesis
The architecture is very flexible and also it tries to use as little computational power as possible since the package used are already big. The flexibility is obtained by dividing in three nodes all the parts of the architecture: one node to communicate with the aruco detection package, one node to communicate with the ontology and one node that implements the state machine. If there is the need to change one part of this architecture, for example the hint is not retrieved by aruco anymore but it is retrieved through a proximity sensor, for example, it would be sufficent to change the node that implements the interface with the aruco_ros package. For what concerns the computational power I delayed the update of the map and I decided to do as little computation as possible in the code I implemented. 

## System's Features
To implement the navigation part I decided to use the move_base algortihm by changing some parameters in the configuration files in order to obtain a better behaviour in this environment.  
I decided to look for hints also while moving from one room to the other and not only inside the room to have better probability of finding all of them. Also in the unfortunate case that when the robot has visited all rooms one or more hints are still missing the state machine resets the visited rooms and starts again searching in the rooms in random order.  
The choice of looking in the rooms in random order has been  made to avoid having always the same behavior, to observe different scenario and how the robot would behave.

## System's Limitations
The main limitation of this architecture is the impossibility of checking if all the hints from one room have been retrieved or if there are some missing. In my architecture I can retrive a hint only once and since I can receive hints also while moving between rooms I have no way of associating the hint to the specific room.  
Another possible limitation is the speed of the simulation, probably due to the heavy computational load the simulation is very slow and sometimes the move_base algortihm takes a while to recognize it has reached the goal position. To complete the entire simulation my computer takes at least 15 minutes. Probably by reducing the resolution of the camera and the impact of the camera and of the move_base algorithm this aspect would improve.

## Possible Technical Improvements
A possible technical improvement would be changing the finite state machine in order to have a state in which the robot is in the room and looks for hints only in that time frame. By doing so it would be possible to recognize if all the hints in a room, we know there are only five hints, are retrieved. This would avoid the need to look in a room more than once and would likely assure a faster completion of the program.  
When a complete and consisten hypothesis has been found the finite state machine first finishes the action in the current state and only then changes to go check if the hypothesis is the winning one. This can be improved by guaranteeing a preemption of all actions and an immediate execution of the behavior needed to check the solution.

# Authors 
Zoe Betta  
s5063114@studenti.unige.it  
zoe.betta@gmail.com
