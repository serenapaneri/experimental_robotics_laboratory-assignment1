#!/usr/bin/env python2

import rospy
import random
from random import randint
from armor_msgs.srv import *
from armor_msgs.msg import * 
# from exprob_ass1.srv import Winhypothesis, WinhypothesisResponse

people = ['Rev. Green', 'Prof. Plum', 'Col. Mustard','Msr. Peacock', 'Miss. Scarlett', 'Mrs. White']
weapons = ['Candlestick', 'Dagger', 'Lead Pipe', 'Revolver', 'Rope', 'Spanner']
places = ['Conservatory', 'Lounge', 'Kitchen', 'Library', 'Hall', 'Study', 'Ballroom', 'Dining room', 'Billiard room']
ID = ['0000', '0001', '0002', '0003', '0004', '0005', '0006', '0007', '0008', '0009']

hypotheses = []
feasible_hypotheses = []
winning_hypothesis = []
armor_interface = None
hint = []

def main():
    global armor_interface
    rospy.init_node('menage_ontology')
    
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    # loading the cluedo ontology
    load()
    # adding the individuals in the TBox
    tbox()
    # reason 
    reasoner()
    # disjoint the individuals of all the classes
    disjoint_individuals()
    
    winning_hypo()
    
    gen_hints()
    
    # save the new modified ontology
    save()
    
    # hypotheses_service = rospy.Service('hypothesis_srv', Hypothesis, hypothesis_handle)
    # winning_hypothesis_service = rospy.Service('winning_hypothesis_srv', Winhypothesis, winning_hypo)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("fail")


def load():
    """
      This function is used to load the ontology
    """
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'LOAD'
    req.primary_command_spec = 'FILE'
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass1/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
    msg = armor_interface(req)
    res = msg.armor_response  


def tbox():
    """
      This function is used to load all the individuals of all classes in the ontology
    """
    global people, weapons, places
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    
    # instances of the class suspected person
    for person in people:
    	req.args = [person, 'PERSON']
    	msg = armor_interface(req)
    	res = msg.armor_response
    print('The suspects have been uploaded in the TBox')
    
    # instances of the class probable implements
    for weapon in weapons:
        req.args = [weapon, 'WEAPON']
        msg = armor_interface(req)
        res = msg.armor_response
    print('The implements have been uploaded in the TBox')
    
    # instances of the class suspected scenes of murder
    for place in places:
        req.args = [place, 'PLACE']
        msg = armor_interface(req)
        res = msg.armor_response
    print('The scenes of murder have been uploaded in the TBox')


def disjoint_individuals():
    """
      This operation needs to be done in order to disjoint the individuals
      from each other 
    """
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'DISJOINT'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['PERSON']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The individuals of the class PERSON have been disjoint')
    req.args = ['WEAPON']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The individuals of the class WEAPON have been disjoint')
    req.args = ['PLACE']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The individuals of the class PLACE have been disjoint')
    
def onto_class(wh):
    if wh == 'who':
        return 'PERSON'
    elif wh == 'what':
        return 'WEAPON'
    elif wh == 'where':
        return 'PLACE'

def reasoner():
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response

def save():
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'SAVE'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    req.args = ['/root/ros_ws/src/exprob_ass1/final_ontology.owl']
    msg = armor_interface(req)
    res = msg.armor_response
    print('The new ontology has been saved under the name final_ontology.owl')
    
def search(list1, list2):
    """
      This function is used to check if the element in the list2 are present or
      not in the list1
    """
    result = any(item in list1 for item in list2)
    if result:
        return True
    else:
        return False
    
def list_index(list1, list2):
    """
      This function is used to search the elements of a list2 in a list1 and
      return their indexes
    """
    check = search(list1, list2)
    if check == True:
        index = [i for i,item in enumerate(list1) if item in list2]
        return index
    else:
        return []
          
    
def all_hypotheses():
    hypotheses = [[] for _ in range(10)]
    
    # complete and consistent hypotheses
    hypotheses[0].append(random.choice(people))
    hypotheses[0].append(random.choice(weapons))
    hypotheses[0].append(random.choice(places))
    hypotheses[0].append('0000')
     
    hypotheses[1].append(random.choice(people))
    hypotheses[1].append(random.choice(weapons))
    hypotheses[1].append(random.choice(places))
    hypotheses[1].append('0001')
    
    hypotheses[2].append(random.choice(people))
    hypotheses[2].append(random.choice(weapons))
    hypotheses[2].append(random.choice(places))
    hypotheses[2].append('0002')
     
    hypotheses[3].append(random.choice(people))
    hypotheses[3].append(random.choice(weapons))
    hypotheses[3].append(random.choice(places))
    hypotheses[3].append('0003')
    
    # uncomplete hypotheses 
    hypotheses[4].append(random.choice(people))
    hypotheses[4].append(random.choice(people))
    hypotheses[4].append(random.choice(places))
    hypotheses[4].append('0004')
    
    hypotheses[5].append(random.choice(weapons))
    hypotheses[5].append(random.choice(places))
    hypotheses[5].append(random.choice(places))
    hypotheses[5].append('0005')
    
    hypotheses[6].append(random.choice(people))
    hypotheses[6].append(random.choice(weapons))
    hypotheses[6].append(random.choice(weapons))
    hypotheses[6].append('0006')
     
    # inconsistent hypotheses
    hypotheses[7].append(random.choice(people))
    hypotheses[7].append(random.choice(weapons))
    hypotheses[7].append(random.choice(weapons))
    hypotheses[7].append(random.choice(places))
    hypotheses[7].append('0007')
    
    hypotheses[8].append(random.choice(people))
    hypotheses[8].append(random.choice(weapons))
    hypotheses[8].append(random.choice(places))
    hypotheses[8].append(random.choice(places))
    hypotheses[8].append('0008')
    
    hypotheses[9].append(random.choice(people))
    hypotheses[9].append(random.choice(people))
    hypotheses[9].append(random.choice(people))
    hypotheses[9].append(random.choice(weapons))
    hypotheses[9].append(random.choice(places))
    hypotheses[9].append(random.choice(places))
    hypotheses[9].append('0009')
    return hypotheses
    
def feasible_hypo():
    hypo = all_hypotheses()
    for i in range(4):
        feasible_hypotheses.append(hypo[i])
    return feasible_hypotheses
    
def winning_hypo():
    feas_hypo = feasible_hypo()
    n = randint(0, len(feas_hypo)-1)
    winning_hypothesis.append(feas_hypo[n])
    print('The winning hypothesis is:')
    print("{} with the {} in the {}".format(winning_hypothesis[0][0], winning_hypothesis[0][1], winning_hypothesis[0][2]))
    load_winning_hypothesis(winning_hypothesis)
    return winning_hypothesis

def flatten(list_):
    flat_list = []
    for element in list_:
        if type(element) is list:
            for item in element:
                flat_list.append(item)
        else:
            flat_list.append(element)
    return flat_list        
    
def load_winning_hypothesis(win):
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'ADD'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['Winning_hypothesis', 'HYPOTHESIS']
    # [name that you want to give, cathegory on the ontology]
    msg = armor_interface(req)
    res = msg.armor_response
    
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['who','Winning_hypothesis', winning_hypothesis[0][0]]
    msg = armor_interface(req)
    res = msg.armor_response 
    
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['what','Winning_hypothesis', winning_hypothesis[0][1]]
    res = armor_interface(req)
    
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['where','Winning_hypothesis', winning_hypothesis[0][2]]
    msg = armor_interface(req)
    res = msg.armor_response 
    
    print("The solution of the game has been uploaded")
    
def complete():
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['COMPLETE']
    msg = armor_interface(req)
    res = msg.armor_response 
    
def inconsistent():
    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'QUERY'
    req.primary_command_spec = 'IND'
    req.secondary_command_spec = 'CLASS'
    req.args = ['INCONSISTENT']
    msg = armor_interface(req)
    res = msg.armor_response 
    
def gen_hints():
    random_hypo = []
    hypo = all_hypotheses()
    random_hypo.append(random.choice(hypo[:]))
    print(random_hypo)
    flat_hypo = flatten(random_hypo)
    # extracting all the indexes (if they exist)
    people_index = list_index(flat_hypo, people)
    weapons_index = list_index(flat_hypo, weapons)
    places_index = list_index(flat_hypo, places)
    id_index = list_index(flat_hypo, ID)
    hints = [[] for _ in range(len(flat_hypo) - 1)]
    print(people_index)
    print(weapons_index)
    print(places_index)
    print(id_index)
    i = 0
    for item in id_index:
        for a in people_index:
            if people_index != []:
                hints[i].append(flat_hypo[a])
                hints[i].append(flat_hypo[item]) # questo fisso
                for b in weapons_index:
                    if weapons_index != []:
                        hints[i + 1].append(flat_hypo[b])
                        hints[i + 1].append(flat_hypo[item])
                        for c in places_index:
                            if places_index != []:
                                hints[i + 2].append(flat_hypo[c])
                                hints[2].append(flat_hypo[item])
                                print(hints)



if __name__ == '__main__':
    main()
