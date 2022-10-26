#!/usr/bin/env python2

import rospy
from exprob_ass1.msg import Hint
from exprob_ass1.srv import Hypothesis

def generate_hints():
    rospy.wait_for_service('menage_ontology')
    print('Waiting for the ontology service')
    try:
        ontology_client = rospy.ServiceProxy('hypothesis_srv', Hypothesis)
        res = HypothesisResponse
        res.
    
    
    

def main():
    rospy.init_node('hints')
    
    hint_pub = rospy.Publisher('hint_generator', Hint)
    r = rospy.Rate(5)
    
    msg = Hint()
    msg.ID = 
    msg.ind = 
    msg.wh = 
    
    while not rospy.is_shoutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    main()
