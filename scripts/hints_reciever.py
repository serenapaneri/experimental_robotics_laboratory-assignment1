#! /usr/bin/env python2

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
    
def main():
    global hints
    rospy.init_node('hints_reciever')
    hint_sub = rospy.Subscriber('hint', Hint, hint_callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    # rospy.wait_for_message('hint', Hint)
    # rospy.wait_for_message('hint', Hint)
    # rospy.wait_for_message('hint', Hint)
        print(hints)
    # rospy.spin()
    
if __name__ == '__main__':
    main()
