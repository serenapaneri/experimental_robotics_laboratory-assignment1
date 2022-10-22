#!/usr/bin/env python

import rospy
import random
from random import randint

people = ['Rev. Green', 'Prof. Plum', 'Col. Mustard','Msr. Peacock', 'Miss. Scarlett', 'Mrs. White']
weapons = ['Candlestick', 'Dagger', 'Lead Pipe', 'Revolver', 'Rope', 'Spanner']
places = ['Conservatory', 'Lounge', 'Kitchen', 'Library', 'Hall', 'Study', 'Ballroom', 'Dining room', 'Billiard room']
ID = ['0001','0002','0003']

hypotheses = []
feasible_hypotheses = []
winning_hypothesis = []

def main():
    rospy.init_node('menage_ontology')
    all_hypothesis()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("fail")


def search(list_):
    result = any(item in list_ for item in ID)
    if result:
        return True
    return False
                
def list_index(list1, list2):
    index = [i for i,item in enumerate(list1) if item in list2]
    return index

def all_hypothesis():

    hypotheses = [[] for _ in range(4)]

    hypotheses[0].append(random.choice(weapons))
    hypotheses[0].append(random.choice(places))
    hypotheses[0].append(random.choice(people))
    hypotheses[0].append('0000')
    
    hypotheses[1].append(random.choice(weapons))
    hypotheses[1].append(random.choice(places))
    hypotheses[1].append(random.choice(people))
    hypotheses[1].append('0001')
    
    hypotheses[2].append(random.choice(weapons))
    hypotheses[2].append(random.choice(places))
    hypotheses[2].append(random.choice(people))
    hypotheses[2].append('0002')
    
    hypotheses[3].append(random.choice(weapons))
    hypotheses[3].append(random.choice(weapons))
    hypotheses[3].append(random.choice(places))
    hypotheses[3].append(random.choice(places))
    hypotheses[3].append(random.choice(people))
    hypotheses[3].append('0003')
    
                    
    print(hypotheses) 
    
    for i in range(3):
        feasible_hypotheses.append(hypotheses[i])
        
    print(feasible_hypotheses)
    
    # qui sbaglio
    print(len(feasible_hypotheses))
    n = randint(0, len(feasible_hypotheses) - 1)
    print(n)
    
    winning_hypothesis.append(feasible_hypotheses[n])
        
    print(winning_hypothesis)
    print(winning_hypothesis[0][0])
    print(winning_hypothesis[0][1])
    print(winning_hypothesis[0][2])
    
                        
    
if __name__ == '__main__':
    main()
