#!/usr/bin/env python2

import rospy
import random
from exprob_ass1.msg import Hint

hypo = rospy.get_param('hypo')
people = rospy.get_param('people')
weapons = rospy.get_param('weapons')
places = rospy.get_param('places')
ID = rospy.get_param('ID')
random_hypo = []
indexes = []

def main():
    global hypo, people, weapons, places
    rospy.init_node('hints') 

    random_hypo.append(random.choice(hypo[:]))
    flat_hypo = flatten(random_hypo)
    print(flat_hypo)
    
    # extracting id index
    id_index = list_index(flat_hypo, ID)
    
    hints = [[] for _ in range(len(flat_hypo) - 1)]
    for i in range(len(flat_hypo) - 1):
        for j in id_index:
            hints[i].append(flat_hypo[i])
            hints[i].append(flat_hypo[j])
            print(hints)
        
    
    # msg = Hint()
    # msg = who.flat_hypo[]
    # msg = what.flat_hypo[]
    # msg = where.flat_hypo[]
    # hint_pub.publish(msg)
    
    rospy.spin()
    
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
