#! /usr/bin/env python2

## @package exprob_ass1
#
# \file state_machine.py
# \brief script that implements the state machine of the game.
#
# \author Serena Paneri
# \version 1.0
# \date 11/11/2022
# \details
#
# Subscribes to: <BR>
#     Hint
#
# Publishes to: <BR>
#     None
#
# Serivces: <BR>
#     None
#
# Client Services: <BR>
#     armor_interface_srv
#     winning_hypothesis
#     comm
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In this node there is a state machine built using the smach package.
#     There are three different states in which the robot can be:
#     - Motion: in this state there is the simulation of the motion of the robot within the
#               rooms of the game. Here there are two types of check. In the first one the
#               number of hints collected is evaluated. If it is not sufficienet the robot
#               keeps moving, instead if they are enough, there is a second check.
#               This check evaluates if the hypothesis composed by the hints found is 
#               a complete and consistent hypothesis. If it is not the robot should restart
#               again the process.
#     - Room: in this state the robot simulates the collection of hints, a new hint each
#             time it is in a new room. 
#     - Oralce: in this final state the robot asks to the oracle if the complete and 
#               consistent hypothesis that it has found is the correct one. If it is then
#               the game is finished, otherwise the robot restarts the process searching for+
#               new hints.


import rospy
import random
import smach
import smach_ros
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exprob_ass1.msg import Hint
from exprob_ass1.srv import Winhypothesis, WinhypothesisRequest, Command, CommandRequest

# waiting for the mange_ontology node
time.sleep(3)
# retrieving the values previously stored in the parameter server
people = rospy.get_param('people')
weapons = rospy.get_param('weapons')
places = rospy.get_param('places')
ID = rospy.get_param('ID')

# hint subscriber
hint_sub = None
# armor client
armor_interface = None
# oracle client
oracle_client = None
# command client
comm_client = None

hints = []
hypo = []
url = ''

# counters
hint_count = 0
attempt = 0


##
# \brief The reasoner of the ontology.
# \param: None
# \return: None
#
# This function implements the reasoner of the ontology that needs to be started in order to update
# the knowledge of the ontology.
def reasoner():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response


##
# \brief It is the apply command of armor.
# \param: None
# \return: None
#
# This function apply the changes done to the ontology.  
def apply_():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'APPLY'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response


##
# \brief It is the query command to retrieve an individual from a class.
# \param: None
# \return: res
#
# This functions returns, if there are any, the individuals of the class COMPLETED of the ontology.
def complete():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['COMPLETED']
    msg = armor_interface(req)
    res = msg.armor_response
    return res


##
# \brief It is the query command to retrieve an individual from a class.
# \param: None
# \return: res
#
# This functions returns, if there are any, the individuals of the class INCONSISTENT of the ontology.  
def inconsistent():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['INCONSISTENT']
    msg = armor_interface(req)
    res = msg.armor_response
    return res


##
# \brief It saves the changes on a new ontology file.
# \param: None
# \return: None
#
# This functions saves the ontology in a new file, also saving the inferences.
def save():

    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'SAVE'
    req.primary_command_spec = 'INFERENCE'
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass1/hhhhhhh.owl']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The new ontology has been saved under the name final_ontology_inferred.owl')
    

##
# \brief It select a random room.
# \param: None
# \return: prev
#
# This functions chooses a random room from the list places and make sure that the one randomly
# selected is always different from the previous one.
def room_choice():

    global places
    prev = None
    room = random.choice(places)
    if room != prev:
        prev = room
    return prev


##
# \brief It is the callback of the hint subscriber.
# \param: msg
# \return: hint
#
# This functions stores in the hints list the messages recieved from the hint publisher.   
def hint_callback(msg):

    global hints
    hints.append(msg.ind)
    hints.append(msg.ID)
    hints.append(msg.dim)
    return hints


##
# \brief This function search an element in a list.
# \param: list_, element
# \return: True, False
#
# This functions checks if a specific element is present (True) or not (False) in a list.     
def search(list_, element):
    """
      This function check if an element is present or not into a list
    """
    for i in range(len(list_)):
        if list_[i] == element:
            return True
    return False


##
# \brief This function search an element in a list.
# \param: element
# \return: hypo
#
# This function is used to store the elements found into a list in order to have them sorted,
# since the hints recieved are not.      
def classes(element):

     global people, weapons, places, hypo
     if search(people, element) == True:
         hypo.insert(0, element)
     elif search(weapons, element) == True:
         hypo.insert(1, element)
     elif search(places, element) == True:
         hypo.insert(2, element)
     return hypo
    

##
# \brief Function that upload the individuals of the hypothesis.
# \param: hypo_
# \return: None
#
# This function uploads each individual of the hypothesis in the ontology of the game. 
# In particular this function is able to upload whatever hypothesis (complete and consistent,
# uncomplete or inconsistent).
def upload_hypothesis(hypo_):

    global people, weapons, places, attempt
    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'

    for element in hypo_:
        # if the element of the list is in people list
        who = search(people, element)
        # if the element of the list is in weapons list
        what = search(weapons, element)
        # if the element of the list is in places list
        where = search(places, element)
        if who == True:  
            req.command = 'ADD'
            req.primary_command_spec = 'OBJECTPROP'
            req.secondary_command_spec = 'IND'
            req.args = ['who','Hypothesis' + str(attempt), element]
            msg = armor_interface(req)
            res = msg.armor_response 
    
        elif what == True:
            req.command = 'ADD'
            req.primary_command_spec = 'OBJECTPROP'
            req.secondary_command_spec = 'IND'
            req.args = ['what','Hypothesis' + str(attempt), element]
            res = armor_interface(req)
    
        elif where == True:
            req.command = 'ADD'
            req.primary_command_spec = 'OBJECTPROP'
            req.secondary_command_spec = 'IND'
            req.args = ['where','Hypothesis' + str(attempt), element]
            msg = armor_interface(req)
            res = msg.armor_response 
    
    print("The hypothesis {} has been uploaded".format(attempt))
     

##
# \brief Class Motion of the state_machine.
# \param: None
# \return: None
#
# This class should simulate the movement of the robot between the various rooms of the cluedo game.
# If the hints percieved are less than the expected number given by the subscriber, then the 
# robot should keep going searching hints in other rooms.
# If, instead, the number of hints collected is the right one the the class should check if the 
# hypothesis formed with those hints is complete and consistent.
# If it's not the robot should restart searching for collecting other hints, instead, if the 
# hypothesis is complete and consistent the robot can go to the oracle room trying its guessing.
# There are three outcomes:
# - enter room, if the robot is still collecting hints
# - go_oracle, if the hypothesis formulate is complete and consistent
# - moving, if the hypothesis formulated is uncomplete or inconsistent.  
class Motion(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['enter_room','go_oracle', 'motion'])
        
    def execute(self, userdata):
    
        global hypo, attempt, comm_client, hint_count, hint_sub, url
        # choosing a random room
        random_room = room_choice()
        # waiting for the message hint
        rospy.wait_for_message('hint', Hint)

        # if the number of hints collected is less than expected 
        if hint_count < hints[-1]:
            time.sleep(2)
            # the robot continues to search 
            print("The robot is going to the {} ..".format(random_room))
            time.sleep(5)
            return 'enter_room'

        # if the robot completed to collect hints
        else:
            # counter that keeps track of the hypotheses collected
            attempt += 1
            # upload the hypothesis into the ontology 
            upload_hypothesis(hypo)
            time.sleep(2)
            # apply
            apply_()
            # reason
            reasoner()
            # client that advertise the server with the command 'stop'
            comm_client('stop')
            time.sleep(3)
            
            # url that is recieved from the query of the ontology
            url = '<http://www.emarolab.it/cluedo-ontology#Hypothesis{}>'.format(attempt)
            
            # check completeness and consistency
            print('Checking if it is complete ..')
            iscomplete = complete()
            time.sleep(1)
            # if the list of queried object of the class COMPLETED is empty
            if len(iscomplete.queried_objects) == 0:
                print('The hypothesis is uncomplete')
                # reset the hint counter
                hint_count = 0
                # empty the list hypo
                hypo.clear()
                # client that advertise the server with the command 'start'
                comm_client('start')
                time.sleep(1)
                return 'motion'
            # if the list of queried object of the class COMPLETED is not empty
            elif len(iscomplete.queried_objects) != 0:
                # checking if the current hypothesis is present or not in the list of queried objects
                # of the class COMPLETE
                if url not in iscomplete.queried_objects:
                    print('The hypothesis is uncomplete')
                    # reset the hint counter
                    hint_count = 0
                    # empty the list hypo
                    hypo.clear()
                    # client that advertise the server with the command 'start'
                    comm_client('start')
                    time.sleep(1)
                    return 'motion'
                elif url in iscomplete.queried_objects:
                    print('The hypothesis is complete')
                    time.sleep(1)
                
                    print('Checking if it is consistent ..')
                    isinconsistent = inconsistent()
                    # if the list of queried object of the class INCONSISTENT is not empty
                    if len(isinconsistent.queried_objects) != 0:
                        # checking if the current hypothesis is present or not in the list of queried 
                        # objects of the class INCONSISTENT
                        if url in isinconsistent.queried_objects:
                            print('The hypothesis is inconsistent')
                            # reset the hint counter
                            hint_count = 0
                            # empty the list hypo
                            hypo.clear()
                            # client that advertise the server with the command 'start'
                            comm_client('start')
                            time.sleep(1)
                            return 'motion'
                        elif url not in isinconsistent.queried_objects:
                            print('The hypothesis is complete and consistent')
                            time.sleep(1)
                            print('The robot is ready to go to the oracle')
                            return 'go_oracle'
                    # if the list of queried object of the class INCONSISTENT is empty
                    elif len(isinconsistent.queried_objects) == 0:
                        print('The hypothesis is complete and consistent')
                        time.sleep(1)
                        print('The robot is ready to go to the oracle')
                        return 'go_oracle'


##
# \brief Class Room of the state_machine.
# \param: None
# \return: None
#
# This class should simulate what happens when the robot enters in a room searching for hints.
# Each time the robot is in this state it simulates the collection of a single hints, keeping
# track of the number of hints collected in this attempt.
# Here there is the motion outcome.     
class Room(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion'])
        
    def execute(self, userdata):
    
        global hint_sub, hints, hint_count, hypo
        print("The robot is looking for hints ..")
        time.sleep(3)
        # counter that keeps track of the hints recieved
        hint_count += 1
        # consider the third-last element of the list that corresponds to the last individual added
        print('HINT FOUND: {}'.format(hints[-3]))
        # assigning the individual found into the right position since they come in a random order
        classes(hints[-3])
        time.sleep(2)
        return 'motion'


##
# \brief Class Oracle of the state_machine.
# \param: None
# \return: None
#
# This class should simulate what happens when the robot has collected all the hints that forms
# a complete and consistent hypothesis, and it tries its guess.
# This class should inform the robot if the hypothesis found is the winning one or not, and this
# is done thanks to the oracle_service that checks the ID of the hypothesis of the robot with
# the winning one.
# If the hypothesis is correct then the game is finished. If it is not then the robot should
# restart searching for new hints and repeat all the process.
# Here we have two outcomes:
# - game_finished, if the hypothesis found is the correct one
# - motion, if the hypothesis found is wrong.     
class Oracle(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion','game_finished'])
        
    def execute(self, userdata):
    
        global oracle_client, comm_client, hints, hypo, attempt, hint_count
        print('The robot is inside the oracle rooom')
        time.sleep(3)
        
        # waiting for the service that gives the ID of the winning hypothesis
        rospy.wait_for_service('winning_hypothesis')
        
        print('Oracle: "Name your guess"')
        time.sleep(2)
        print('{} with the {} in the {}'.format(hypo[0], hypo[1], hypo[2]))
        time.sleep(1)
        # oracle client
        req = WinhypothesisRequest()
        # knowing that the ID is always in the second position
        req.ID = hints[-2]
        res = oracle_client(req)
        # if the two strings coincides
        if res.check == True:
            print('Yes, you guessed right!')
            save()
            return 'game_finished'
        # otherwise if they are not the same   
        elif res.check == False:
            print('No you are wrong, maybe next time you will have better luck')
            comm_client('start')
            # emtpty the list
            hint_count = 0
            hypo.clear()
            time.sleep(5)
            return 'motion' 
        

##
# \brief This is the main function of the node state_machine.
# \param: None
# \return: None
#
# In the main function the node state_machine is initialized and here the service are called
# as well as the hint subscriber.
def main():

    global armor_interface, oracle_client, comm_client, hint_sub
    rospy.init_node('state_machine')
    sm = smach.StateMachine(outcomes=['game_finished'])
    
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    
    # armor client
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    # oracle client
    oracle_client = rospy.ServiceProxy('winning_hypothesis', Winhypothesis)
    # command client
    comm_client = rospy.ServiceProxy('comm', Command)
    # hint subscriber
    hint_sub = rospy.Subscriber('hint', Hint, hint_callback)

    with sm:
        smach.StateMachine.add('Motion', Motion(), 
                               transitions={'enter_room':'Room', 
                                            'go_oracle':'Oracle',
                                            'motion': 'Motion'})
        smach.StateMachine.add('Room', Room(), 
                               transitions={'motion':'Motion'})
        smach.StateMachine.add('Oracle', Oracle(), 
                               transitions={'motion':'Motion', 
                                            'game_finished':'game_finished'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()

