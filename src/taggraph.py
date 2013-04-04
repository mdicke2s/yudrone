#!/usr/bin/env python

#
#  taggraph.py
#  Part of the yudrone project.
#
#  This file builds a state machine dealing with the high level state transitions
#  involved in the graph traversal task.
#  
#
#  Created by Andrew Speers and Michael Dicke on 13-04-03.
#  Copyright (c) 2013 Andrew Speers and Michael Dicke. All rights reserved.
#

import roslib; roslib.load_manifest('yudrone'); roslib.load_manifest('smach_ros')
from flight import *
from ar_recog.msg import Tags, Tag
from geometry_msgs.msg import Twist
from yudrone.msg import commandsMsg
from std_msgs.msg import Int32

import rospy
import smach
import smach_ros

# define state
class GetNode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_node','acquired_node'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GET');
        if False:
            return 'no_node'
        else:
            return 'acquired_node'

class SearchNode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['still_lost','acquired_node'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH');
        if False:
            return 'still_lost'
        else:
            return 'acquired_node'

class TargetApproachNode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['still_tracking','in_range','lost'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state TARGET_APPROACH');
	return 'still_tracking'

class DockNode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['docked','lost'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state DOCKING');
        if True:
            return 'docked'
        else:
            return 'lost'

class HoverNode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state HOVER');
        return 'finished'


# main
def main():
    rospy.init_node('drone_network_traverse_state_machine')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['landed', 'gave_up'])
    
    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('GET', GetNode(), 
                               transitions={'no_node':'GET', 
                               'acquired_node':'SEARCH'})
        smach.StateMachine.add('SEARCH', SearchNode(), 
                               transitions={'still_lost':'SEARCH',
                               'acquired_node':'TARGET_APPROACH'})
        smach.StateMachine.add('TARGET_APPROACH', TargetApproachNode(), 
                               transitions={'still_tracking':'TARGET_APPROACH',
                               'in_range':'DOCKING',
                               'lost':'SEARCH'})        
        smach.StateMachine.add('DOCKING', DockNode(), 
                               transitions={'docked':'HOVER',
                               'lost':'SEARCH'})
        smach.StateMachine.add('HOVER', HoverNode(), 
                               transitions={'finished':'GET'})
    
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
