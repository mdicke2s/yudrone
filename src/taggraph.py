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
from yudrone.msg import commandsMsg, commandStatus
from std_msgs.msg import Int32

import rospy
import smach
import smach_ros

import abc

# define general state
# comes up with message passing for yudrone_command
class communicatingState(smach.State):
    __metaclass__ = abc.ABCMeta
    def __init__(self):
	self.pubCommands = rospy.Publisher('yudrone/commands', commandsMsg)
	self.subCommandStatus = rospy.Subscriber( "yudrone/cmdStatus", commandStatus, self.handleStatus )
	rospy.loginfo("communicationState initialized")
	rospy.sleep(0.3)

    @abc.abstractmethod
    def handleStatus(self, msg):
      # receive status information and response
      return
  
class GetNode(communicatingState):
    def __init__(self):
        communicatingState.__init__(self)
        smach.State.__init__(self, outcomes=['no_node','acquired_node'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GET');
        if False:
            return 'no_node'
        else:
            return 'acquired_node'
            
    def handleStatus(self, msg):
      return

class SearchNode(communicatingState):
    def __init__(self):
        communicatingState.__init__(self)
        smach.State.__init__(self, outcomes=['still_lost','acquired_node'])
        self.searchIsRunning = False
        self.commandID = -1	
        
        m = commandsMsg()
        self.pubCommands.publish(m) # send empty message as initialization
    
    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH');
        self.hasAcquiredNode = False
        
        # execute search command
        if self.searchIsRunning == False and self.hasAcquiredNode == False:
	  m=commandsMsg()
	  m.hasSearch=True
	  m.tagNr=11
	  self.pubCommands.publish(m)
	  self.commandID = m.header.seq
	  rospy.loginfo("search command submitted: id = %i"%self.commandID)
	  self.searchIsRunning = True
	
	# busy wait for result
        while (True):
	  rospy.sleep(0.5)
	  if self.hasAcquiredNode == False:
            return 'still_lost'
          else:
            return 'acquired_node'
    
    def handleStatus(self, msg):
	rospy.loginfo("Status changed: (id=%i) %s"%(msg.id, msg.status))
	if msg.id == -1 and msg.status == "available":
	  self.searchIsRunning = False
        if msg.id == self.commandID and msg.status == "done_successfully":
	  self.hasAcquiredNode = True

class TargetApproachNode(communicatingState):
    def __init__(self):
        communicatingState.__init__(self)
        smach.State.__init__(self, outcomes=['still_tracking','in_range','lost'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state TARGET_APPROACH');
	return 'in_range'
            
    def handleStatus(self, msg):
      return

class DockNode(communicatingState):
    def __init__(self):
        communicatingState.__init__(self)
        smach.State.__init__(self, outcomes=['docked','lost'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state DOCKING');
        if True:
            return 'docked'
        else:
            return 'lost'
            
    def handleStatus(self, msg):
      return

class HoverNode(communicatingState):
    def __init__(self):
        communicatingState.__init__(self)
        smach.State.__init__(self, outcomes=['finished'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state HOVER');
        return 'finished'
            
    def handleStatus(self, msg):
      return


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
    
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('yudrone_states', sm, 'yodrone/stateMachine')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
