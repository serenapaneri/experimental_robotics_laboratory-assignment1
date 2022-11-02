#!/usr/bin/env python2

import rospy
import random
import time
from exprob_ass1.msg import Hint

hints = []
hint_sub = None

def hint_callback(msg):
    global hints
    # print("{} with {}".format(msg.ind, msg.ID))
    hints.append(msg.ind)
    hints.append(msg.ID)
    # print(hints)
    return hints
    
def wait_msg(num):
    rospy.wait_for_message('hint', Hint)
    
def main():
    global hints
    rospy.init_node('hints_reciever')
    hint_sub = rospy.Subscriber('hint', Hint, hint_callback)
    dim = rospy.get_param('dim')
    count = 0
    for n in range(dim):
        # count += 1
        wait_msg(n)
        print(hints)
        # print(hints[-2])
        print('aspetta')

    
if __name__ == '__main__':
    main()
