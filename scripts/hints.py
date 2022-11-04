#!/usr/bin/env python2

import rospy
import random
from exprob_ass1.msg import Hint
from exprob_ass1.srv import Command

hypo = rospy.get_param('hypo')
ID = rospy.get_param('ID')
hint_pub = None
comm_service = None
random_hypo = []

start = True

def com(req):
    if (req.command == 'start'):
        start = True
    elif (req.command == 'stop'):
        start = False
    return True

def main():
    global hypo, ID, hint_pub, comm_service
    rospy.init_node('hints') 
    
    # hints publisher
    hint_pub = rospy.Publisher('hint', Hint)
    comm_service = rospy.Service('comm', Command, com)
    
    rate = rospy.Rate(1)
    
    # dovrei aggiungere un controllo qui
    # random hypotheses from the one generates
    
    if start == True:
        random_hypo.append(random.choice(hypo[:]))
        flat_hypo = flatten(random_hypo)
        print(flat_hypo)
    
        # extracting id index
        id_index = list_index(flat_hypo, ID)
    
        # generate the hints of the random hypothesis
        hints = [[] for _ in range(len(flat_hypo) - 1)]
        for i in range(len(flat_hypo) - 1):
            for j in id_index:
                hints[i].append(flat_hypo[i])
                hints[i].append(flat_hypo[j])
                print(hints)
        
        rospy.set_param('dim', len(hints))
        rospy.sleep(5)  
        while not rospy.is_shutdown():
            msg = Hint()
            for i in range(len(hints)):
                msg.ind = hints[i][0]
                msg.ID = hints[i][1]
    
                # rospy.loginfo(msg)
                rate.sleep()
                hint_pub.publish(msg)
                
    elif start == False:
        while not rospy.is_shutdown():
            print('restarting')
            rate.sleep()

def flatten(list_):
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
