#!/usr/bin/env python2

import rospy
import random
from random import randint
from armor_msgs.srv import *
from armor_msgs.msg import * 
# from exprob_ass1.srv import Winhypothesis

armor_interface = None
feasible_hypotheses = []
winning_hypothesis = []

# DEVO IMPLEMENTARE IL SERVICE MA FUNZIONA

def main():
    global armor_interface
    rospy.init_node('Oracle')
    hypo = rospy.get_param('hypo')
    print(hypo)
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)

    # selecting the feasible hypotheses of the game
    for i in range(4):
        feasible_hypotheses.append(hypo[i])
        
    n = randint(0, len(feasible_hypotheses)-1)
    
    # random winning hypothesis
    winning_hypothesis.append(feasible_hypotheses[n])

    print('The winning hypothesis is:')
    print("{} with the {} in the {}".format(winning_hypothesis[0][0], winning_hypothesis[0][1], winning_hypothesis[0][2]))
    load_winning_hypothesis(winning_hypothesis)

    rospy.spin()

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


if __name__ == '__main__':
    main()
