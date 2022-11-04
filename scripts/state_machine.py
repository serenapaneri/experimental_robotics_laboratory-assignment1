#! /usr/bin/env python2

import rospy
import random
import smach
import smach_ros
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exprob_ass1.msg import Hint
from exprob_ass1.srv import Winhypothesis, WinhypothesisRequest, Command, CommandRequest

people = rospy.get_param('people')
weapons = rospy.get_param('weapons')
places = rospy.get_param('places')

dim = rospy.get_param('dim')
hints = []
hypo = []
hint_sub = None
armor_interface = None
oracle_client = None
comm_client = None
hint_count = 0
attempt = 0

def reasoner():
    """
      It is the reasoner of the ontology
    """
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response
    return res

def complete():
    """
      This function checks if the hypothesis is complete
    """
    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['COMPLETE']
    msg = armor_interface(req)
    res = msg.armor_response
    
def inconsistent():
    """
      This function checks if the hypothesis is inconsistent
    """
    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['INCONSISTENT']
    msg = armor_interface(req)
    res = msg.armor_response

def save():
    """
      It saves the ontology in a new file called final_ontology
    """
    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'SAVE'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass1/ddd.owl']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The new ontology has been saved under the name final_ontology.owl')
    

def room_choice():
    """
      This function chooses a random room
    """
    rooms_list = ['Conservatory', 'Lounge', 'Kitchen', 'Library', 'Hall', 'Study', 'Ballroom', 'Dining Room', 'Billiard Room'] 
    prev = None
    room = random.choice(rooms_list)
    if room != prev:
        prev = room
    return prev
    
def hint_callback(msg):
    """
      This function is the callback of the subscriber hint_sub
    """
    global hints
    # print("{} with {}".format(msg.ind, msg.ID))
    hints.append(msg.ind)
    hints.append(msg.ID)
    return hints
    
def search(list_, element):
    """
      This function check if an element is present or not into a list
    """
    for i in range(len(list_)):
        if list_[i] == element:
            return True
    return False
    
def classes(element):
     """
       This function is used to store the elements found into a list in
       order to have them sorted, since the hints recieved are not.
     """
     global people, weapons, places, hypo
     if search(people, element) == True:
         hypo.insert(0, element)
     elif search(weapons, element) == True:
         hypo.insert(1, element)
     elif search(places, element) == True:
         hypo.insert(2, element)
     return hypo
     
def upload_hypothesis(hypo_):
    """
      This function upload the hypothesis in the ontology
    """
    global people, weapons, places
    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['hypothesis' + str(attempt), 'HYPOTHESIS']
    msg = armor_interface(req)
    res = msg.armor_response
    
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
            req.args = ['who','hypothesis' + str(attempt), element]
            msg = armor_interface(req)
            res = msg.armor_response 
    
        elif what == True:
            req.command = 'ADD'
            req.primary_command_spec = 'OBJECTPROP'
            req.secondary_command_spec = 'IND'
            req.args = ['what','hypothesis' + str(attempt), element]
            res = armor_interface(req)
    
        elif where == True:
            req.command = 'ADD'
            req.primary_command_spec = 'OBJECTPROP'
            req.secondary_command_spec = 'IND'
            req.args = ['where','hypothesis' + str(attempt), element]
            msg = armor_interface(req)
            res = msg.armor_response 
    
    print("The hypothesis {} has been uploaded".format(attempt))
     
    
class Motion(smach.State):
    # this class should simulate the movement between rooms
    # if the hint that I have percieved are less than 3 then the robot shold keep going 
    # searching for hints in the other rooms
    # instead, if it has collect 3 hints, building a complete hypothesis and this hypothesis 
    # is consistent then it should go to the oracle testing his hypothesis 
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['enter_room','go_oracle'])
        
    def execute(self, userdata):
        global hypo, attempt, comm_client
        # choosing a random room
        random_room = room_choice()

        # if the number of hints collected is less than expected 
        if hint_count < dim:
            time.sleep(2)
            # the robot continues to search 
            print("The robot is going to the {} ..".format(random_room))
            time.sleep(5)
            return 'enter_room'

        else:
            # comando per stoppare hints.py
            comm_client("stop")
            # counter that keeps track of the hypotheses collected
            attempt += 1
            # we upload the hypothesis into the ontology 
            upload_hypothesis(hypo)
            # reason
            reasoner()
            # save()
            time.sleep(5)
            # check completeness and consistency
            print('Checking if it is complete ..')
            time.sleep(1)
            # iscomplete = complete()
            # if iscomplete.success == False:
            print('The hypothesis is uncomplete')
                # change hypothesis
            # elif iscomplete.success == True:
            print('The hypothesis is complete')
            #    time.sleep(1)
            print('Checking if it is consistent ..')
            #     time.sleep(1)
            #    isincosistent = inconsistent()
            #    if isincosistent.success == False:
            print('The hypothesis is inconsistent')
            #         time.sleep(1)
            #     elif isincosistent.success == True:
            print('The hypothesis is complete and consistent')
            print('The robot is ready to go to the oracle')
            return 'go_oracle'
        
        
class Room(smach.State):
    # this class should simulates what happens when the robot enters in a room searching for
    # hints. 
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion'])
        
    def execute(self, userdata):
        global hint_sub, dim, hints, hint_count, hypo
        print("The robot is looking for hints ..")
        time.sleep(3)
        # subsccriber to the hint topic
        hint_sub = rospy.Subscriber('hint', Hint, hint_callback)
        rospy.wait_for_message('hint', Hint)
        # counter that keeps track of the hints recieved
        hint_count += 1
        # consider the second-last element of the list that corresponds to the last individual added
        print('HINT FOUND: {}'.format(hints[-2]))
        # assigning the hint into the right position since they come in a random order
        classes(hints[-2])
        time.sleep(2)
        return 'motion'
        
class Oracle(smach.State):
    # this class should simulate what happens when the robot has already collect 3 hints
    # that makes a complete hypothesis and moreover it has to be consistent (we already
    # have to check that). So this have to tell the robot if the hypothesis that it has 
    # brought is the correct one or not. If it is then the game is over, if it is not the
    # game restart and the robot should searching for the correct hinst again.  
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
        print('{} with {} in the {}'.format(hypo[0], hypo[1], hypo[2]))
        time.sleep(1)
        req = WinhypothesisRequest()
        # knowing that the ID is always in the second position also hints[1] and hints[3] gives the same result
        req.ID = hints[5]
        res = oracle_client(req)
        # if the two strings coincides
        if res.check == True:
            print('Yes, you guessed right!')
            save()
            return 'game_finished'
        # otherwise if they are not the same   
        elif res.check == False:
            print('No you are wrong, maybe next time you will have better luck')
            # emtpty the list
            hint_count = 0
            hypo.clear()
            save()
            # comando per far ripartire lo script hints.py
            comm_client('start')
            time.sleep(5)
            return 'motion' 
        
        
def main():
    global armor_interface, oracle_client, comm_client
    rospy.init_node('state_machine')
    sm = smach.StateMachine(outcomes=['game_finished'])
    
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    oracle_client = rospy.ServiceProxy('winning_hypothesis', Winhypothesis)
    comm_client = rospy.ServiceProxy('comm', Command)
    

    with sm:
        smach.StateMachine.add('Motion', Motion(), 
                               transitions={'enter_room':'Room', 
                                            'go_oracle':'Oracle'})
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

