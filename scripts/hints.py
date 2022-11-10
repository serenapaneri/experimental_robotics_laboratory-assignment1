#!/usr/bin/env python2

import rospy
import random
import time
from exprob_ass1.msg import Hint
from exprob_ass1.srv import Command

hypo = rospy.get_param('hypo')
ID = rospy.get_param('ID')
hint_pub = None
comm_service = None
random_hypo = []
start = True

def com(req):
    global start
    if (req.command == 'start'):
        start = True
    elif (req.command == 'stop'):
        start = False
    return start

def main():
    """
      This is the main function of the hints node.
      It chooses a random hypothesis from the ones avaiable and extracts
    """
    global hypo, ID, hint_pub, comm_service, start
    rospy.init_node('hints') 
    
    # hints publisher
    hint_pub = rospy.Publisher('hint', Hint)
    # command service
    comm_service = rospy.Service('comm', Command, com)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # if the command recieved is 'start'
        if start == True:
            # random hypotheses from the one generates
            random_hypo.append(random.choice(hypo[:]))
            flat_hypo = flatten(random_hypo)

            # extracting id index
            id_index = list_index(flat_hypo, ID)
    
            # generate the hints of the random hypothesis
            hints = [[] for _ in range(len(flat_hypo) - 1)]
            for i in range(len(flat_hypo) - 1):
                for j in id_index:
                    hints[i].append(flat_hypo[i])
                    hints[i].append(flat_hypo[j])
                    print(hints)

            # storing the dimention of the actual dimension of the hints to be recieved
            # IT IS NOT THE ACTUAL DIMENSION
            # rospy.set_param('dim', len(hints))
            dim = len(hints)
            rospy.sleep(5)  
            while start == True:
                # publishing each hint generated from the current random hypothesis
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
                random_hypo.clear()
                rate.sleep()


def flatten(list_):
    """
      This function trasforms a nested list into a flat list
    """
    flat_list = []
    for element in list_:
        if type(element) is list:
            for item in element:
                flat_list.append(item)
        else:
            flat_list.append(element)
    return flat_list
    
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


if __name__ == '__main__':
    main()
