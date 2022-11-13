# First Assignment of the Experimental Robotics Laboratory course (Robotics Engineering/ JEMARO, Unige)

## General infomration
This ROS package contains the implementation of the first assignment of the Experimental Robotics Laboratory. The aim of this project is to implement a vanilla version of the cluedo game. 
Indeed there is a robot that simulates, just as in the game, the search for a murderer, the murder weapon and the crime scene.
The robot, during its research, moves around the rooms of the cluedo house with the purpuse of collecting clues to solve the mistery. Indeed, collecting these hints it is able to formulate hypotheses, thus trying to find the winning one that will be revealed by the oracle of the game (the envelope of the real game).
Moreover, it is asked to implement the behavioral software architecture of the project and it is also required to deal with an ontology.

## Software architecture
### Component diagram
![Alt text](/images/comp_diagram.jpg?raw=true)
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
![Alt text](/images/state_diagram.jpg?raw=true)
In the state diagram are shown all the states in which the robot could be and moreover the outcomes of the various state. 
Thete is the Motion state in which the robot should simulate the movments around the rooms of the cluedo house. Moreover in this state the robot checks how many hints have been collected in the current attempt and, after collecting all the hints, thus forming an hypothesis, the completeness and the consistency of the hypothesis is checked. 
If the hypothesis is uncomplete or inconsistent, the outcome of the state is the state itself. Instead if the hypothesis is complete and consistent, the outcome of the state is the Oracle.
However, if the robot has not collected enough hints to create an hypothesis the outcome of the state is Room.
In the room state the robot simply simulate the collection of hints: each time it is in this state, a new hint is collected, and the only outcome is the motion state.
In the Oracle state the oracle checks if the ID of the current hypothesis coincides with the winning one, if it is not the outocomes is Motion and the process of searching starts all over again. Instead if the two IDs coincides the robot goes into the final state that is game_finished.

### Temporal diagram
![Alt text](/images/temp_diagram.jpg?raw=true)
In the temporal diagram is possible to see how the other nodes are called and menage by the smach state machine. 
Indeed, at first the state machine asks for hints, that are exctracted from a random hypothesis, whose in turn, has been selected by the whole lists of hypotheses generated in the menage_ontology node. After the request of the state machine the hints are published through the hints publisher of the node hints.
After collecting all the hints, thus composing an hypothesis, the state_machine asks the ARMOR service to check if the current hypothesis is complete and consistent. 
If the hypothesis is complete and consistent, then the state_machine asks to the oracle node to check, through the service winning_hypothesis, if the ID of the current hypothesis and the ID of the winning one, coincides or not.

### Messages, Services and RosParameters
#### Messages
In the msg folder you can find the [**Hint.msg**](https://github.com/serenapaneri/exprob_ass1/tree/main/msg/Hints.msg) message.
The message, that is sent thanks to the hint publisher, has the following structure: 
string ind
string ID
int64 dim
So, the message contains an idividual of the current hypothesis, the ID associated to the current hypothesis, and the number of hints that have been extctracted from that hypothesis.
#### Services
In the srv folder you can find Command.srv and Winhypothesis.srv.
- [**Command.srv**](https://github.com/serenapaneri/exprob_ass1/tree/main/srv/Command.srv): The structure is the one below 
string command
---
bool ok
So, it contains the request done by the client that is a string, and the response is a bool.
- [**Winhypothesis.srv**](https://github.com/serenapaneri/exprob_ass1/tree/main/srv/Winhypothesis.srv): The structure is the one below
string ID
---
bool check
This service is implemented in the oracle node and simply check if two strings (IDs) coincides or not.

#### RosParameters
Within the code you can notice the use of five ROS parameter server.
- people : in this parameter server the list of possible murders is stored
- weapons : in this parameter server the list of possible murder weapons is stored
- places : in this parameter server the list of possible scene of crime is stored
- ID : in this parameter server the list of all the ID, each one associated to one hypothesis, is stored
- hypo : in this parameter server is contained the list of the random hypothesis selected for the current game session.

## Installation and running procedures
### Display robot's behavior

## Behavior of the package
![Alt text](/images/motion.png?raw=true)
![Alt text](/images/room.png?raw=true)
![Alt text](/images/oracle.png?raw=true)
![Alt text](/images/game_finished.png?raw=true)

![Alt text](/images/uncomplete.png?raw=true)
![Alt text](/images/inconsistent.png?raw=true)
![Alt text](/images/attempt.png?raw=true)
![Alt text](/images/winning_hypo.png?raw=true)


## Working hypothesis and environment

### System's features

### System's limitations

### Possible technical improvements



## Author and contact
Serena Paneri

4506977

s4506977@studenti.unige.it
