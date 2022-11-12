# First Assignment of the Experimental Robotics Laboratory course (Robotics Engineering/ JEMARO, Unige)

## General infomration
This ROS package contains the implementation of the first assignment of the Experimental Robotics Laboratory. The aim of this project is to implement a vanilla version of the cluedo game. 
Indeed there is a robot that simulates, just as in the game, the search for a murderer, the murder weapon and the crime scene.
The robot, during its research, moves around the rooms of the cluedo house with the purpuse of collecting clues to solve the mistery. Indeed, collecting these hints it is able to formulate hypotheses, thus trying to find the winning one that will be revealed by the oracle of the game (the envelope of the real game).
Moreover, it is asked to implement the behavioral software architecture of the project and it is also required to deal with an ontology.

## Software architecture
### Component diagram
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/component_diagram.jpg)
With the component diagram it is possible to see the overall behavior and how the whole architecture is organized.
In this diagrams are shown, besides the armor service, the four nodes of which the package is composed.

- [**menage_ontology**](https://github.com/serenapaneri/exprob_ass1/tree/main/scripts/menage_ontology.py): In this ROS node are defined all the list of individuals, divided in people, weapons and places, as well as a list containing all the possible ID associated to each hypothesis of the game. This lists are then stored into a parameter server.
This node contains a client for the ARMOR service, to be able to menage the cluedo_ontology.
In particular, the operation performed, thanks to this service, are the loading of the cluedo_ontology, the upload of the TBox of the game containing all the individuals, the disjoint of the individuals belonging to each class and the starting of the reasoner of the ontology.
Moreover in this node the hypotheses of the game are generated randomly, some of them are complete and consistent, some are uncomplete and some are complete but inconsistent. These hypotheses are then stored in a parameter server.
- [**hints**](https://github.com/serenapaneri/exprob_ass1/tree/main/scripts/hints.py): In this ROS node the publisher of the hints and the command service are implemented, in order to send the hints in an orderly way.
After retrieving the hypotheses of the game from the ros parameter server, the hypotheses of the game are selected in a pseudo-random way. Indeed the list of hypotheses is shuffled and each time that the command client asks for a new set of hints, a new hypothesis is taken from that list. From the current hypothesis, all the hints contained are extracted and sent, via ROS message, thanks to the hint publisher. The ROS message, besides the individuals and the ID of the current hypothesis, also contains the number of hints extracted from that specific hypothesis in order to advertise the state_machine about the number of hints that needs to be collected. 
- [**oracle**](https://github.com/serenapaneri/exprob_ass1/tree/main/scripts/oracle.py): This ROS node contains the armor client and the winning_hypothesis service. After retrieving the hypotheses from the ROS parameter, an hypothesis is randomly picked from the feasible ones (meaning complete and consistent hypotheses) and that will be the winning hypothesis of the game. In particular the winning_hypothesis service is implemented that is used to check if the ID of the winning hypothesis is the same or not of the current hypothesis of the game.
Then the winning hypothesis is loaded in the ontology, thanks to the armor service.
- [**state_machine**](https://github.com/serenapaneri/exprob_ass1/tree/main/scripts/state_machine.py): In this node a state machine built with the smach package is implemented.
There are three different states in which the robot could be:
    - Motion: This python class should simulate the movement of the robot between the various rooms of the cluedo game. If the hints percieved are less than the expected number given by the subscriber, then the robot should keep going searching hints in other rooms. If, instead, the number of hints collected is the right one the the class should check if the hypothesis formed with those hints is complete and consistent. If it's not the robot should restart searching for collecting other hints, instead, if the hypothesis is complete and consistent the robot can go to the oracle room trying its guessing.
    - Room: This python class should simulate what happens when the robot enters in a room searching for hints. Each time the robot is in this state it simulates the collection of a single hints, keeping track of the number of hints collected in this attempt.
    - Oracle: This python class should simulate what happens when the robot has collected all the hints that forms a complete and consistent hypothesis, and it tries its guess. This class should inform the robot if the hypothesis found is the winning one or not, and this is done thanks to the oracle_service that checks the ID of the hypothesis of the robot with the winning one. If the hypothesis is correct then the game is finished. If it is not then the robot should restart searching for new hints and repeat all the process.
- armor_service: Thanks to the armor service it is possible to 'interact' with the cluedo ontology performing some operations like loading and saving the ontology, uploading the hints beloging to each hypothesis, disjoint the individuals, performing the query and starting the reasoner of the ontology. 

### State diagram
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/state_diagram.jpg)

### Temporal diagram
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/temporal_diagram.jpg)

### Messages, Services and RosParameters



## Installation and running procedures
### Display robot's behavior

## Behavior of the package
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/motion.png)
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/room.png)
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/oracle.png)
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/game_finished.png)

![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/uncomplete.png)
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/inconsistent.png)
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/attempt.png)
![alt text](https://github.com/serenapaneri/exprob_ass1/tree/main/images/winning_hypo.png)


## Working hypothesis and environment

### System's features

### System's limitations

### Possible technical improvements



## Author and contact
Serena Paneri

4506977

s4506977@studenti.unige.it
