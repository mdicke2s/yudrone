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
#
#  Note: To share data among states we use the superclass ydState and static variables.
#        Smach userdata (out) is used anyway, to display data in smach gui

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
	
        # initialize tag order
        ydState.tagNrList=(1, 6)
        ydState.tagAltdList=(500,700)
        ydState.tagPointer = -1

    def getTagNr(self):
      return ydState.tagNrList[ydState.tagPointer]
      
    def getTagAltd(self):
      return ydState.tagAltdList[ydState.tagPointer]
	
    def setToNextTag(self):
      ydState.tagPointer = (ydState.tagPointer + 1) % 2 # modulo number of tags in list
      rospy.loginfo('current tagNr is %i'%self.getTagNr())
    
    @abc.abstractmethod
    def handleStatus(self, msg):
      # receive status information and response
      return
      
    @abc.abstractmethod
    def clean(self):
      # reset state variables
      # intended for transitions that exit the state to go to another state
      return
      
    def checkCmdNode(self):
      cmdNodeRunning = (self.pubCommands.get_num_connections() > 0)
      if cmdNodeRunning == False:
	rospy.logerr('Command node is not running')
      return cmdNodeRunning
  
''' define getNode state ******************************************************'''
class GetNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['no_node','acquired_node', 'land', 'take_off'], output_keys=['tagNr'])      

    def execute(self, userdata):
	rospy.sleep(0.5)
	self.checkCmdNode()
        if False:
	  self.clean()
	  return 'no_node'
        else:
	  self.setToNextTag()
	  userdata.tagNr = self.getTagNr()
	  self.clean()
	  return 'acquired_node'
            
    def handleStatus(self, msg):
      return
      
    def clean(self):
      return
      
''' define TakeOff state ******************************************************'''
class TakeOffNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['taking_off','in_flight'], output_keys=['tagNr'])  

    def execute(self, userdata):
	userdata.tagNr = self.getTagNr()
	self.checkCmdNode()
        if True:
	  self.clean()
	  return 'in_flight'
        else:
          return 'taking_off'
            
    def handleStatus(self, msg):
      return
    
    def clean(self):
      return
      
''' define Landing state ******************************************************'''
class LandingNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['landing','landed'], output_keys=['tagNr'])   

    def execute(self, userdata):
	userdata.tagNr = self.getTagNr()
	self.checkCmdNode()
        if False:
            return 'landing'
        else:
	    self.clean()
            return 'landed'
            
    def handleStatus(self, msg):
      return
    
    def clean(self):
      return

''' define searchNode state ******************************************************'''
class SearchNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['still_lost','acquired_node', 'gave_up'], output_keys=['tagNr'])
        self.clean() 
        m = commandsMsg()
        self.pubCommands.publish(m) # send empty message as initialization
    
    '''
    submits search command
    '''
    def execute(self, userdata):
	userdata.tagNr = self.getTagNr()
	self.checkCmdNode()
        self.hasAcquiredNode = False
        self.giveUp = False
        
        # execute search command
        if self.searchIsRunning == False and self.hasAcquiredNode == False and self.giveUp == False:
	  rospy.sleep(0.5)
	  m=commandsMsg()
	  m.hasSearch=True
	  m.tagNr=self.getTagNr()
	  m.searchStartAltd = self.getTagAltd()
	  self.pubCommands.publish(m)
	  self.commandID = m.header.seq
	  rospy.loginfo("search command submitted: id = %i"%self.commandID)
	  self.searchIsRunning = True
	if self.searchIsRunning == True and self.hasAcquiredNode == False:
	  rospy.loginfo("still searching for tag nr %i"%self.getTagNr())
	
	rospy.sleep(0.5)
	if self.hasAcquiredNode == True:
          self.clean()
          return 'acquired_node'
        elif self.giveUp == True:
	  self.clean()
	  return 'gave_up'
        else:
          return 'still_lost'
    
    '''
    - give_up if search was unsuccessful for two times 
    - acuired_node if search finished successfully
    '''
    def handleStatus(self, msg):
      rospy.loginfo("Command status changed: (id=%i) %s"%(msg.id, msg.status))
      if (msg.id == self.commandID and msg.status == "search not successful") or \
	 (msg.id == -1 and msg.status == "available" and self.searchIsRunning == True):
        # search was finished without success
        self.searchIsRunning = False
        self.searchTrys = self.searchTrys + 1
        if self.searchTrys > 1:
	  self.giveUp = True
      if msg.id == self.commandID and msg.status == "search successful":
        # search was finished successfully
        self.hasAcquiredNode = True

    def clean(self):
      self.searchIsRunning = False
      self.hasAcquiredNode = False
      self.giveUp = False
      self.searchTrys = 0
      self.commandID = -1

''' define faceNode state ******************************************************'''
class FaceNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['lost','done', 'control'], output_keys=['tagNr'])
        self.clean()
        
        m = commandsMsg()
        self.pubCommands.publish(m) # send empty message as initialization
	
    '''
    submits face command
    '''
    def execute(self, userdata):
	userdata.tagNr = self.getTagNr()
	self.checkCmdNode()
        # execute face command
        if self.faceIsRunning == False and self.isFacing_stable == False and self.isLost == False:
	  m=commandsMsg()
	  m.hasFace=True
	  m.tagNr=self.getTagNr()
	  self.pubCommands.publish(m)
	  self.commandID = m.header.seq
	  rospy.loginfo("face command submitted: id = %i"%self.commandID)
	  self.faceIsRunning = True
	if self.faceIsRunning == True and self.isFacing_stable == False:
	  rospy.loginfo("still attempting to face tag nr %i"%self.getTagNr())
	
	rospy.sleep(0.5)
	if self.isLost == True:
	  self.clean()
	  return 'lost'
	elif self.isFacing_stable == True:
	  #send out release message
	  m = commandsMsg()
	  m.hasRelease = True
	  self.pubCommands.publish(m)
          self.clean()
          return 'done'
        else:
          return 'control'
    
    '''
    submits release command as soon as face became stable
    '''
    def handleStatus(self, msg):
	rospy.loginfo("Command status changed: (id=%i) %s"%(msg.id, msg.status))
	if (msg.id == -1 and msg.status == "available" and self.faceIsRunning == True) or \
	   (msg.id == self.commandID and msg.status == "released" and self.faceIsRunning == True):
	  # face was finished without success
	  self.faceIsRunning = False
	  self.isLost = True
        if msg.status == "face stable":
	  # search was finished successfully
	  self.isFacing_stable = True
	  
    def clean(self):
      self.faceIsRunning = False
      self.commandID = -1
      self.isLost = False # need to search for tag
      self.isFacing_stable = False # within error parameters and ready to finish facing command
      
''' define TargetApproachNode state ******************************************************'''
class TargetApproachNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['still_tracking','in_range','lost'], output_keys=['tagNr'])
        self.clean()
    
    '''
    executes
    * approach which brings drone closer
    * altd(+500)
    * horizontal(x=100)
    '''
    def execute(self, userdata):
	userdata.tagNr = self.getTagNr()
	self.checkCmdNode()
	if self.approachState == "initialized": # call approach
	  self.approachState = "approach running"
	  m=commandsMsg()
	  m.hasApproach=True
	  m.tagNr=self.getTagNr()
	  self.pubCommands.publish(m)
	  self.commandID = m.header.seq
	  rospy.loginfo("approach command submitted: id = %i"%self.commandID)
	elif self.approachState == "approach done": # call altitude
	  self.approachState = "altitude running"
	  m=commandsMsg()
	  m.hasAltdDelta=True
	  m.altdDelta=500
	  self.pubCommands.publish(m)
	  self.commandID = m.header.seq
	  rospy.loginfo("altitude command submitted: id = %i"%self.commandID)
	elif self.approachState == "altitude done": # call horizontal
	  self.approachState = "horizontal running"
	  m=commandsMsg()
	  m.hasHorizontal=True
	  m.horizontalX=100
	  m.horizontalY=0
	  self.pubCommands.publish(m)
	  self.commandID = m.header.seq
	  rospy.loginfo("horizontal command submitted: id = %i"%self.commandID)	  
	elif self.approachState == "horizontal done":
	  self.clean()
	  return 'in_range'
	rospy.loginfo("Approchestate = %s"%self.approachState)
	#rospy.sleep(0.2)
	return 'still_tracking'
            
    def handleStatus(self, msg):
      if msg.status == "approach done" and self.approachState == "approach running":
	self.approachState = "approach done"
      if msg.status == 'altitude command sucessfully' and self.approachState == "altitude running":
	self.approachState = "altitude done"
      if (msg.id == self.commandID and msg.status == 'done' and self.approachState == "horizontal running") or \
	 (msg.status == 'available'and self.approachState == "horizontal running"):
	self.approachState = "horizontal done"

    def clean(self):
      self.commandID = -1
      #self.approachState = "initialized"
      self.approachState = "horizontal done" # skip complete approach state
      
''' define DockNode state ******************************************************'''
class DockNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['docked','lost', 'docking'], output_keys=['tagNr'])
    
    def execute(self, userdata):
	userdata.tagNr = self.getTagNr()
	self.checkCmdNode()
        if True:
            return 'docked'
        else:
            return 'lost'
            
    def handleStatus(self, msg):
      return
      
    def clean(self):
      return

''' define HoverNode state ******************************************************'''
class HoverNode(ydState):
    def __init__(self):
        ydState.__init__(self)
        smach.State.__init__(self, outcomes=['finished', 'hovering'])
    
    def execute(self, userdata):
	self.checkCmdNode()
        return 'finished'
            
    def handleStatus(self, msg):
      return

    def clean(self):
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
    #key = 'y'
    #while key == 'y':
    outcome = sm.execute()
    rospy.loginfo('State Machine finsihed with "%s".'%outcome)
    #  key = raw_input('State Machine finished with "%s". Do you want to restart [y/n]'%outcome)

if __name__ == '__main__':
  try:
    main()
  except:
    print 'Exception occured'
    rospy.signal_shutdown(str(sys.exc_info()))
    for line in sys.exc_info():
      print(line)
