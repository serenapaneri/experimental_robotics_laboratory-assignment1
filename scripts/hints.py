#!/usr/bin/env python2

## @package exprob_ass1
#
# \file hints.py
# \brief script to publish the hints of the cluedo game and implements a service to send them in a orderly way.
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
#     hint
#
# Serivces: <BR>
#     comm
#
# Client Services: <BR>
#     None
#
# Action Services: <BR>
#     None
#
# Description: <BR>
#     In the hints node the hints pubisher and the comm service are implemented to send the hints
#     in an orderly way. The hypotheses of the game are selected in a pseudo-random way and each
#     hints generated from that hypothesis is sended thanks to the hints publisher, that also sends
#     the number of hints generated from the actual hypothesis.
#     All the process is menaged by the comm_service that can stop the process of sending the hints
#     of the chosen hypothesis and can restart the process with a new hypothesis from which the hints
#     will be collected and then sended.


import rospy
import random
import time
from exprob_ass1.msg import Hint
from exprob_ass1.srv import Command

# hint publisher
hint_pub = None
# command service
comm_service = None

random_hypo = []
start = True


##
# \brief Callback of the comm_service.
# \param: req, CommandRequest
# \return: start
#
# This function recieves the command from the client in the state_machine node. When the client commands to
# start then the start variable is set to True and when it commands to stop the start variable is set to False.
def com(req):

    global start
    if (req.command == 'start'):
        start = True
    elif (req.command == 'stop'):
        start = False
    return start


##
# \brief Main function of the node hints where the node is initialized and the hint publisher and the comm 
#        service are implemented.
# \param: None
# \return: None
#
# This is the main function of the hints node and here are implemented the hints pubisher and the comm
# service, in order to send the hints in an orderly way. The elements of the lists hypo, that has been
# retrieved from the parameter server, are shuffled in order to be selected in a pseudo-random way.
# With this approached all the hypotheses generated can be selected but without any repetition.
# Given an hypothesis then all the hints (of that specific hypothesis) are then sended thanks to the
# publisher with a form like [individuals, ID]. Moreover, also the number of hints generated are published.
# When all the hints of that hypothesis are recieved from the state_machine, then the clients asks to stop
# sending the hints of that hypothesis, and when it asks to restart the process a new hypothesis is picked
# and new hints are sended.
def main():

    global hint_pub, comm_service, start, random_hypo
    # initializing the hints node
    rospy.init_node('hints') 
    
    # hints publisher
    hint_pub = rospy.Publisher('hint', Hint)
    # command service
    comm_service = rospy.Service('comm', Command, com)
    
    rate = rospy.Rate(1)
    
    # waiting for menage_ontology node
    time.sleep(3)
    
    # retrieving the values previously stored in the parameter server
    hypo = rospy.get_param('hypo')
    ID = rospy.get_param('ID')
    
    # mixing the elements of hypo to generate a random order
    random.shuffle(hypo)
    
    while not rospy.is_shutdown() and len(hypo) > 0:
        # if the command recieved is 'start'
        if start == True:
            # picking the last element of the mixed list
            random_hypo.append(hypo[-1])
            # delete the last element of the mixed list
            hypo.pop()
            # flattening the hypothesis
            flat_hypo = flatten(random_hypo)
            # extracting id index
            id_index = list_index(flat_hypo, ID)
    
            # generate the hints of the random hypothesis
            hints = [[] for _ in range(len(flat_hypo) - 1)]
            for i in range(len(flat_hypo) - 1):
                for j in id_index:
                    hints[i].append(flat_hypo[i])
                    hints[i].append(flat_hypo[j])

            # number of hints generated 
            dim = len(hints)
            rospy.sleep(5)
             
            while start == True:
                # publishing each hint generated from the current random hypothesis and
                # also the number of hints sent
                msg = Hint()
                for i in range(dim):
                    msg.ind = hints[i][0]
                    msg.ID = hints[i][1]
                    msg.dim = dim
    
                    # rospy.loginfo(msg)
                    rate.sleep()
                    hint_pub.publish(msg)
        
        # if the command recieved is 'stop'        
        elif start == False:
                time.sleep(5)
                # empty the list random_hypo
                random_hypo.clear()
                rate.sleep()


##
# \brief Function that flat a list of list.
# \param: list_
# \return: flat_list
#
# This function trasforms a nested list into a flat list.
def flatten(list_):

    flat_list = []
    for element in list_:
        if type(element) is list:
            for item in element:
                flat_list.append(item)
        else:
            flat_list.append(element)
    return flat_list


##
# \brief Function that checks if elements of list2 exist in list1.
# \param: list1, list2
# \return: True, False
#
# This function is used to check if the elements in the list2 are present or not in the list1.
def search(list1, list2):

    result = any(item in list1 for item in list2)
    if result:
        return True
    else:
        return False


##
# \brief Function that returns the indixes in a list.
# \param: list1, list2
# \return: index, []
#
# This function is used to search the elements of a list2 in a list1 and return their indexes.  
def list_index(list1, list2):

    check = search(list1, list2)
    if check == True:
        index = [i for i,item in enumerate(list1) if item in list2]
        return index
    else:
        return []


if __name__ == '__main__':
    main()
