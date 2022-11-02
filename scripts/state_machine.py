#! /usr/bin/env python2

import rospy
import random
import smach
import smach_ros
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exprob_ass1.msg import Hint
from exprob_ass1.srv import Winhypothesis, WinhypothesisRequest

people = rospy.get_param('people')
weapons = rospy.get_param('weapons')
places = rospy.get_param('places')

dim = rospy.get_param('dim')
hints = []
hypo = []
hint_sub = None
armor_interface = None
hint_count = 0
attempt = 0

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
    req.args = ['/root/ros_ws/src/exprob_ass1/final_ontology.owl']
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
     
def load_hypothesis(hypo_):
    req = ArmorDirectiveReq()
    req.client_name = 'state_machine'
    req.reference_name = 'cluedontology'
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['hypothesis' + str(attempt), 'HYPOTHESIS']
    # [name that you want to give, cathegory on the ontology]
    msg = armor_interface(req)
    res = msg.armor_response
    
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['who','hypothesis' + str(attempt), hypo_[0]]
    msg = armor_interface(req)
    res = msg.armor_response 
    
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['what','hypothesis' + str(attempt), hypo_[1]]
    res = armor_interface(req)
    
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['where','hypothesis' + str(attempt), hypo_[2]]
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
        random_room = room_choice()

        if hint_count < dim:
            time.sleep(2)
            print("The robot is going to the {}".format(random_room))
            time.sleep(5)
            return 'enter_room'

        else:
            # check completeness and consistency
            print('Check if it is complete')
            print('Check if it is consistent')
            time.sleep(5)
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
        print("The robot is looking for hints")
        rospy.sleep(3)
        hint_sub = rospy.Subscriber('hint', Hint, hint_callback)
        # wait_msg()
        rospy.wait_for_message('hint', Hint)
        hint_count += 1
        # prendo sempre l'ultimo ind
        print('Hint found: {}'.format(hints[-2]))
        classes(hints[-2])
        rospy.sleep(2)
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
        global oracle_client, hints, hypo, attempt
        print('The robot is inside the oracle rooom')
        rospy.sleep(5)
        rospy.wait_for_service('winning_hypothesis')
        print('Oracle: "Name your guess"')
        attempt += 1
        time.sleep(2)
        print('{} with {} in the {}'.format(hypo[0], hypo[1], hypo[2]))
        load_hypothesis(hypo)
        time.sleep(1)
        req = WinhypothesisRequest()
        req.ID = hints[5]
        res = oracle_client(req)
        if res.check == True:
            print('Yes, you guessed right')
            save()
            return 'game_finished'
        elif res.check == False:
            print('No you are wrong, maybe next time you will have better luck')
            return 'motion' 
        
        
def main():
    global armor_interface, oracle_client
    rospy.init_node('state_machine')
    sm = smach.StateMachine(outcomes=['game_finished'])
    
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    oracle_client = rospy.ServiceProxy('winning_hypothesis', Winhypothesis)

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

