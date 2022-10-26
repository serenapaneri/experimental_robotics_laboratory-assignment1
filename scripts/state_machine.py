#! /usr/bin/env python2

import rospy
import random
import smach
import smach_ros
import time
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exprob_ass1.msg import Hint
from exprob_ass1.srv import Hypothesis

hints = None

def room_choice():
    rooms_list = ['Conservatory', 'Lounge', 'Kitchen', 'Library', 'Hall', 'Study', 'Ballroom', 'Dining Room', 'Billiard Room'] 
    prev = None
    room = random.choice(rooms_list)
    if room != prev:
        prev = room
    return prev
    
    
class Motion(smach.State):
    # this class should simulate the movement between rooms
    # if the hint that I have percieved are less than 3 then the robot shold keep going 
    # searching for hints in the other rooms
    # instead, if it has collect 3 hints, building a complete hypothesis and this hypothesis 
    # is consistent then it should go to the oracle testing his hypothesis 
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['enter_room','go_oracle'])
        
    def execute(self, userdata):
        random_room = room_choice()
        
        if len(hints) < 3: # implement a counter
            print("The robot is going to the {}".format(random_room))
            time.sleep(5) # simulated time to reach a room
            return 'enter_room'
        else:
            # here check also if it is complete and consistent
            # also after having a feasible hypothesis go to the oracle
            return 'go_oracle'
        
        
class Room(smach.State):
    # this class should simulates what happens when the robot enters in a room searching for
    # hints. 
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['motion'])
        
    def execute(self, userdata):
        print("The robot is looking for hints")
        
        # here the should be the hint subscriber and the counter
        # for the hints recieved

        
        return 'motion'
        
class Oracle(smach.State):
    # this class should simulate what happens when the robot has already collect 3 hints
    # that makes a complete hypothesis and moreover it has to be consistent (we already
    # have to check that). So this have to tell the robot if the hypothesis that it has 
    # brought is the correct one or not. If it is then the game is over, if it is not the
    # game restart and the robot should searching for the correct hinst again.  
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['motion','game_finished'])
        
    def execute(self, userdata):
        
        time.sleep(5)
        rospy.loginfo('Executing state UNLOCKED (users = %f)'%userdata.unlocked_counter_in)
        userdata.unlocked_counter_out = userdata.unlocked_counter_in + 1
        return 'game_finished'
        
        return 'motion' 
        
        
def main():
    rospy.init('state_machine')
    sm = smach.StateMachine(outcomes=['game_finished'])
    
    rospy.wait_for_service('armor_interface_srv')
    print('Waiting for the armor service')
    rospy.wait_for_service('menage_ontology')
    print('Waiting for the ontology service')
    rospy.wait_for_service('hints')
    print('Waiting for the hint service')
    
    armor_interface = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    hypothesis_client = rospy.ServiceProxy('hypothesis_srv', Hypothesis)

    with sm:
        smach.StateMachine.add('Motion', Motion(), 
                               transitions={'enter_room':'Room', 
                                            'go_oracle':'Oracle'})
        smach.StateMachine.add('Room', Room(), 
                               transitions={'motion':'Motion'})
        smach.StateMachine.add('ORACLE', Oracle(), 
                               transitions={'motion':'Motion', 
                                            'game_finished':'game_finished'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()
    











class Unlocked(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['push','coin'],
                             input_keys=['unlocked_counter_in'],
                             output_keys=['unlocked_counter_out'])
        
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(5)
        rospy.loginfo('Executing state UNLOCKED (users = %f)'%userdata.unlocked_counter_in)
        userdata.unlocked_counter_out = userdata.unlocked_counter_in + 1
        return user_action()
    

# define state Locked
class Locked(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['push','coin'],
                             input_keys=['locked_counter_in'],
                             output_keys=['locked_counter_out'])
        self.sensor_input = 0
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        while not rospy.is_shutdown():  
            time.sleep(1)
            if self.sensor_input < 5: 
                rospy.loginfo('Executing state LOCKED (users = %f)'%userdata.locked_counter_in)
                userdata.locked_counter_out = userdata.locked_counter_in + 1
                return user_action()
            self.sensor_input += 1
            self.rate.sleep

        
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LOCKED', Locked(), 
                               transitions={'push':'LOCKED', 
                                            'coin':'UNLOCKED'},
                               remapping={'locked_counter_in':'sm_counter', 
                                          'locked_counter_out':'sm_counter'})
        smach.StateMachine.add('UNLOCKED', Unlocked(), 
                               transitions={'push':'LOCKED', 
                                            'coin':'UNLOCKED'},
                               remapping={'unlocked_counter_in':'sm_counter',
                                          'unlocked_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
