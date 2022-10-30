#!/usr/bin/env python2

import rospy
# from exprob_ass1.msg import Hint

def main():
    rospy.init_node('hints')    
    hypo = rospy.get_param('hypo')

    
    # who = hypothesis.who
    # what = hypothesis.what
    # where = hypothesis.where
    # ID = hypothesis.id

    rospy.spin()


if __name__ == '__main__':
    main()
