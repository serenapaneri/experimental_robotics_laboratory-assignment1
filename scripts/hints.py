#!/usr/bin/env python2

import rospy
from exprob_ass1.msg import Hint
from exprob_ass1.msg import Hypothesis

def callback(data):
    print(data.hypo)

# def generate_hints():
#     rospy.wait_for_service('hypothesis_srv')
#     print('Waiting for the ontology service')
#     ontology_client = rospy.ServiceProxy('hypothesis_srv', Hypothesis)
#     hypotheses = ontology_client()

def main():
    rospy.init_node('hints')
    
    hypo_sub = rospy.Subscriber('hypo', Hypothesis, callback)
    # generate_hints()
    # hint_pub = rospy.Publisher('hint_generator', Hint)
    # r = rospy.Rate(5)
    
    # msg = Hint()
    # msg.ID = 
    # msg.ind = 
    # msg.wh = 
    
    # while not rospy.is_shoutdown():
        # rospy.loginfo(msg)
        # pub.publish(msg)
        # r.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
