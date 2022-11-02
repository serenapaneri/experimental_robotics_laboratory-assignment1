#!/usr/bin/env python2

import rospy
import random
from random import randint
from armor_msgs.srv import *
from armor_msgs.msg import * 
from exprob_ass1.srv import Winhypothesis, WinhypothesisResponse

hypo = rospy.get_param('hypo')

armor_interface = None
feasible_hypotheses = []
winning_hypothesis = []


def main():
    global armor_interface, hypo
    rospy.init_node('Oracle')

    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    rospy.Service('winning_hypothesis', Winhypothesis, win_hypo)

    rospy.spin()
    
def win_hypo(req):
    global hypo
    for i in range(4):
        feasible_hypotheses.append(hypo[i])
        
    n = randint(0, len(feasible_hypotheses)-1)
    
    # random winning hypothesis and loaded in the ontology
    winning_hypothesis.append(feasible_hypotheses[n])
    load_winning_hypothesis(winning_hypothesis)
    
    res = WinhypothesisResponse()

    if winning_hypothesis[0][3] == req.ID:
        res.check = True
    else:
        res.check = False        
    return res
    

def load_winning_hypothesis(win):
    req = ArmorDirectiveReq()
    req.client_name = 'oracle'
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
