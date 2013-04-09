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

''' define general state ******************************************************'''
# superclass that comes up with message passing for yudrone_command
# contains static variables to be accessed from all stati
class ydState(smach.State):
    __metaclass__ = abc.ABCMeta
    def __init__(self):
	self.pubCommands = rospy.Publisher('yudrone/commands', commandsMsg)
	self.subCommandStatus = rospy.Subscriber( "yudrone/cmdStatus", commandStatus, self.handleStatus )
	rospy.loginfo("communicationState initialized")
	rospy.sleep(0.3)
	
        # initialize tag order
        ydState.tagList=(11, 18)
        ydState.tagPointer = 0

    def getTagNr(self):
	return ydState.tagList[ydState.tagPointer]
	
    def setToNextTag(self):
	ydState.tagPointer = (ydState.tagPointer + 1) % 2
    
    @abc.abstractmethod
    def handleStatus(self, msg):
      # receive status information and response
      return
  
''' define getNode state ******************************************************'''
class GetNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['no_node','acquired_node', 'land', 'take_off'])
        

    def execute(self, userdata):
        if False:
            return 'no_node'
        else:
            return 'acquired_node'
            
    def handleStatus(self, msg):
      return
      
''' define TakeOff state ******************************************************'''
class TakeOffNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['taking_off','in_flight'])
        

    def execute(self, userdata):
        if True:
            return 'in_flight'
        else:
            return 'taking_off'
            
    def handleStatus(self, msg):
      return
      
''' define Landing state ******************************************************'''
class LandingNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['landing','landed'])
        

    def execute(self, userdata):
        if False:
            return 'landing'
        else:
            return 'landed'
            
    def handleStatus(self, msg):
      return

''' define searchNode state ******************************************************'''
class SearchNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['still_lost','acquired_node', 'gave_up'])
        self.searchIsRunning = False
        self.commandID = -1
        self.giveUp = False
        
        m = commandsMsg()
        self.pubCommands.publish(m) # send empty message as initialization
        
    def execute(self, userdata):
        self.hasAcquiredNode = False
        
        # execute search command
        if self.searchIsRunning == False and self.hasAcquiredNode == False and self.giveUp == False:
	  m=commandsMsg()
	  m.hasSearch=True
	  m.tagNr=self.getTagNr()
	  self.pubCommands.publish(m)
	  self.commandID = m.header.seq
	  rospy.loginfo("search command submitted: id = %i"%self.commandID)
	  self.searchIsRunning = True
	if self.searchIsRunning == True and self.hasAcquiredNode == False:
	  rospy.loginfo("still searching for tag nr %i"%self.getTagNr())
	
	rospy.sleep(0.5)
	if self.giveUp == True:
	  return 'gave_up'
	elif self.hasAcquiredNode == False:
          return 'still_lost'
        else:
          return 'acquired_node'
    
    def handleStatus(self, msg):
	rospy.loginfo("Command status changed: (id=%i) %s"%(msg.id, msg.status))
	if msg.id == -1 and msg.status == "available":
	  # search was finished without success
	  self.searchIsRunning = False
	  self.giveUp = True
        if msg.id == self.commandID and msg.status == "done_successfully":
	  # search was finished successfully
	  self.hasAcquiredNode = True


''' define faceNode state ******************************************************'''
class FaceNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['lost','done', 'control'])
        self.faceIsRunning = False
        self.commandID = -1
        self.isLost = False
        
        m = commandsMsg()
        self.pubCommands.publish(m) # send empty message as initialization
	
    def execute(self, userdata):
        # execute face command
        if self.faceIsRunning == False and self.isFacing == False and self.isLost == False:
	  m=commandsMsg()
	  m.hasFace=True
	  m.tagNr=self.getTagNr()
	  self.pubCommands.publish(m)
	  self.commandID = m.header.seq
	  rospy.loginfo("face command submitted: id = %i"%self.commandID)
	  self.faceIsRunning = True
	if self.faceIsRunning == True and self.isFacing == False:
	  rospy.loginfo("still attempting to face tag nr %i"%self.getTagNr())
	
	rospy.sleep(0.5)
	if self.isLost == True:
	  return 'lost'
	elif self.isFacing == True:
	  #send out reset message
          return 'done'
        else:
          return 'control'
    
    def handleStatus(self, msg):
	rospy.loginfo("Command status changed: (id=%i) %s"%(msg.id, msg.status))
	if msg.id == -1 and msg.status == "available":
	  # face was finished without success
	  self.faceIsRunning = False
	  self.isLost = True
        if msg.id == self.commandID and msg.status == "stable":
	  # search was finished successfully
	  self.isFacing = True
	  
	  
''' define TargetApproachNode state ******************************************************'''
class TargetApproachNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['still_tracking','in_range','lost'])
    
    def execute(self, userdata):
	return 'in_range'
            
    def handleStatus(self, msg):
      return

''' define DockNode state ******************************************************'''
class DockNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['docked','lost', 'docking'])
    
    def execute(self, userdata):
        if True:
            return 'docked'
        else:
            return 'lost'
            
    def handleStatus(self, msg):
      return

''' define HoverNode state ******************************************************'''
class HoverNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['finished', 'hovering'])
    
    def execute(self, userdata):
        return 'finished'
            
    def handleStatus(self, msg):
      return


''' main function ******************************************************'''
def main():
    rospy.init_node('drone_network_traverse_state_machine')
        
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['landed', 'gave_up'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GET', GetNode(), 
                               transitions={'no_node':'GET',
                               'take_off':'TAKEOFF',
                               'land':'LAND',
                               'acquired_node':'SEARCH'})
        smach.StateMachine.add('TAKEOFF', TakeOffNode(), 
                               transitions={'taking_off':'TAKEOFF', 
                               'in_flight':'GET'})
        smach.StateMachine.add('LAND', LandingNode(), 
                               transitions={'landing':'LAND', 
                               'landed':'GET'})
        smach.StateMachine.add('SEARCH', SearchNode(), 
                               transitions={'still_lost':'SEARCH',
                               'acquired_node':'FACE',
                               'gave_up':'gave_up'})
        smach.StateMachine.add('FACE', FaceNode(), 
                               transitions={'lost':'SEARCH',
                               'control':'FACE',
                               'done':'TARGET_APPROACH'})
        smach.StateMachine.add('TARGET_APPROACH', TargetApproachNode(), 
                               transitions={'still_tracking':'TARGET_APPROACH',
                               'in_range':'DOCK',
                               'lost':'SEARCH'})        
        smach.StateMachine.add('DOCK', DockNode(), 
                               transitions={'docked':'HOVER',
                               'docking':'DOCK',
                               'lost':'SEARCH'})
        smach.StateMachine.add('HOVER', HoverNode(), 
                               transitions={'finished':'GET',
                               'hovering':'HOVER'})
    
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('yudrone_states', sm, 'yodrone/stateMachine')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
