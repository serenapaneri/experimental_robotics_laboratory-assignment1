#! /usr/bin/env python2

import rospy
import random
from armor_msgs.srv import *
from armor_msgs.msg import * 

person = []
weapon = []
place = []


def load():
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'LOAD'
    req.primary_command_spec = 'FILE'
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass1/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
    res = armor_interface(req)   
    

def wh():
"""
  No need since in the ontology we already have the definition who == person ..
"""
    # class suspected person
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'QUERY'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['who', ID]
    
    # class probable implements
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'QUERY'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['what', ID]
    
    # class suspected scene of murder
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['where', ID]


def tbox():
"""
  Try to put all the person, weapons and places into an array and then proceed 
  with a for cycle
"""
    global people, weapons, places
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    
    # instances of the class suspected person
    people = ['Rev. Green', 'Prof. Plum', 'Col. Mustard','Msr. Peacock', 'Miss. Scarlett', 'Mrs. White']
    
    for person in people:
    	req.args = [person, 'PERSON']
    	res = armor_interface(req)
    	print('The suspects have been uploaded in the TBox')
    
    # instances of the class probable implements
    weapons = ['Candlestick', 'Dagger', 'Lead Pipe', 'Revolver', 'Rope', 'Spanner']
    for weapon in weapons:
        req.args = [weapon, 'WEAPON']
        res = armor_interface(req)
        print('The implements have been uploaded in the TBox')
    
    # instances of the class suspected scenes of murder
    places = ['Conservatory', 'Lounge', 'Kitchen', 'Library', 'Hall', 'Study', 'Ballroom', 'Dining room', 'Billiard room']
    for place in places:
        req.args = [place, 'PLACE']
        res = armor_interface(req)
        print('The scenes of murder have been uploaded in the TBox')

    
def disjoint_classes():
"""
  No need since in the ontology the cathegory are already disjoint
"""
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'DISJOINT'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['PERSON']
    res = armor_interface(req)
    req.args = ['WEAPON']
    res = armor_interface(req)
    req.args = ['PLACE']
    res = armor_interface(req)
    

def reasoner():
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    res = armor_interface(req)


def complete():
"""
  No need since in the ontology a complete hypothesis is already defined
"""
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['COMPLETED']
    res = armor_interface(req)


def inconsistent():
"""
  No need since in the ontology a complete hypothesis is already defined
"""
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['INCONSISTENT']
    res = armor_interface(req)


def save():
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'SAVE'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass1/final_ontology.owl']
    res = armor_interface(req)




from exprob_ass1.srv import Hypothesis.srv

hypo = []
killer = []
arm = []
room = []
ID = None

def all_hypothesis():
     
     # inconsistent hypotheses
     hypo[0].killer.append(random.choice(people))
     hypo[0].arm.append(random.choice(weapons))
     hypo[0].arm.append(random.choice(weapons))
     hypo[0].arm.append(random.choice(weapons))
     hypo[0].room.append(random.choice(places))
     hypo[0].ID = 'HPO'
     
     hypo[1].room.append(random.choice(places))
     hypo[1].room.append(random.choice(places))
     hypo[1].arm.append(random.choice(weapons))
     hypo[1].arm.append(random.choice(weapons))
     hypo[1].ID = 'HP1'
     
     hypo[2].killer.append(random.choice(people))
     hypo[2].killer.append(random.choice(people))
     hypo[2].arm.append(random.choice(weapons))
     hypo[2].room.append(ramdom.choice(places))
     hypo[2].ID = 'HP2'
     
     # incomplete hypotheses
     hypo[3].killer.append(random.choice(people))
     hypo[3].killer.append(random.choice(people))
     hypo[3].room.append(random.choice(places))
     hypo[3].ID = 'HP3'
     
     hypo[4].room.append(random.choice(places))
     hypo[4].arm.append(random.choice(weapons))
     hypo[4].room.append(random.choice(places))
     hypo[4].ID = 'HP4'
     
     hypo[5].killer.append(random.choice(people))
     hypo[5].arm.append(random.choice(weapons))
     hypo[5].armo.append(random.choice(weapons))
     hypo[5].ID = 'HP5'
     
     # complete and consistent hypotheses
     hypo[6].killer.append(random.choice(people))
     hypo[6].arm.append(random.choice(weapons))
     hypo[6].room.append(random.choice(places))
     hypo[6].ID = 'HP6'
     
     hypo[7].killer.append(random.choice(people))
     hypo[7].arm.append(random.choice(weapons))
     hypo[7].room.append(random.choice(places))
     hypo[7].ID = 'HP7'
     
     hypo[8].killer.append(random.choice(people))
     hypo[8].arm.append(random.choice(weapons))
     hypo[8].room.append(random.choice(places))
     hypo[8].ID = 'HP8'
     
     hypo[9].killer.append(random.choice(people))
     hypo[9].arm.append(random.choice(weapons))
     hypo[9].room.append(random.choice(places))
     hypo[9].ID = 'HP9'
     
def onto_hypo():
    req = ArmorDirectiveReq()
    req.client_name = 'tutorial'
    req.reference_name = 'ontoTest'
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['hypothesis', HYPOTHESIS]





def main():
    rospy.init_node('menage_ontology')
    rospy.wait_for_service('armor_interface_srv')
    armor_iterface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("fail")
    



if __name__ == '__main__':
    main()
