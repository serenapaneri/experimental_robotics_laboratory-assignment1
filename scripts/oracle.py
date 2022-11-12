#!/usr/bin/env python2

## @package exprob_ass1
#
# \file oracle.py
# \brief script that implements a service to reveal the winning hypothesis of the game.
#
# \author Serena Paneri
# \version 1.0
# \date 11/11/2022
# \details
#
# Subscribes to: <BR>
#     None
#
# Publishes to: <BR>
#     None
#
# Serivces: <BR>
#     winning_hypothesis
#
# Client Services: <BR>
#     armor_interface_srv
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In the oracle node the armor client and the winning_hypothesis service are implemented.
#     The winnning_hypothesis service choses an hypothesis and check if the ID of the hypothesis chosen
#     by the client is the same of the one corresponding to the winning one.
#     The winning hypothesis is then loaded in the ontology.


import rospy
import random
import time
from random import randint
from armor_msgs.srv import *
from armor_msgs.msg import * 
from exprob_ass1.srv import Winhypothesis, WinhypothesisResponse

# waiting for the menage_ontology node
time.sleep(3)
# retrieving the values previously stored in the parameter server
hypo = rospy.get_param('hypo')

# armor client
armor_interface = None

feasible_hypotheses = []
winning_hypothesis = []


##
# \brief Main function of the oracle node.
# \param: None
# \return: None
#
# This function is the main function of the node oracle in which the node is initialized, and the 
# armor client and the winning_hypothesis service are implemented.
def main():

    global armor_interface, hypo
    # initializing the oracle node
    rospy.init_node('oracle')

    # armor service
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    
    # winning hypothesis service
    rospy.Service('winning_hypothesis', Winhypothesis, win_hypo)
    # apply the changes to the ontology
    apply_()
    # reason
    reasoner()

    rospy.spin()


##
# \brief Callback function of the winning_hypothesis service.
# \param: req, WinhypothesisRequest
# \return: res, WinhypothesisResponse
#
# This function implements the callback of the winning_hypothesis service that choses an hypothesis
# randomly from the one that are feasible, so both complete and consistent. The winning hypothesis is
# then loaded in the ontology.
# The service winning_hypothesis checks if the ID of the hypothesis chosen by the client is the same
# of the one corresponding to the winning one.
def win_hypo(req):

    global hypo
    # the first four hypothesis are the ones that are complete and consistent
    for i in range(4):
        feasible_hypotheses.append(hypo[i])
        
    n = randint(0, len(feasible_hypotheses)-1)
    
    # random winning hypothesis and loaded in the ontology
    winning_hypothesis.append(feasible_hypotheses[n])
    upload_winning_hypothesis(winning_hypothesis)
    
    # reason
    reasoner()
    
    # if the ID of the random hypothesis coincides with the winning one
    res = WinhypothesisResponse()

    if winning_hypothesis[0][3] == req.ID:
        res.check = True
    else:
        res.check = False        
    return res
    

##
# \brief Function that upload the individuals of the hypothesis.
# \param: win
# \return: None
#
# This function uploads each individual of the hypothesis in the ontology of the game.
def upload_winning_hypothesis(win):

    req = ArmorDirectiveReq()
    req.client_name = 'oracle'
    req.reference_name = 'cluedontology'
    
    # upload the 'who' of the winning hypothesis
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['who','Winning_hypothesis', win[0][0]]
    msg = armor_interface(req)
    res = msg.armor_response 
    
    # upload the 'what' of the winning hypothesis
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['what','Winning_hypothesis', win[0][1]]
    res = armor_interface(req)
    
    # upload the 'where' of the winning hypothesis
    req.command = 'ADD'
    req.primary_command_spec = 'OBJECTPROP'
    req.secondary_command_spec = 'IND'
    req.args = ['where','Winning_hypothesis', win[0][2]]
    msg = armor_interface(req)
    res = msg.armor_response 
    
    print("The solution of the game has been uploaded")


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
# \brief The reasoner of the ontology.
# \param: None
# \return: None
#
# This function implements the reasoner of the ontology that needs to be started in order to update
# the knowledge of the ontology.  
def reasoner():

    req = ArmorDirectiveReq()
    req.client_name = 'menage_ontology'
    req.reference_name = 'cluedontology'
    req.command = 'REASON'
    req.primary_command_spec = ''
    req.secondary_command_spec = ''
    msg = armor_interface(req)
    res = msg.armor_response
    

if __name__ == '__main__':
    main()
