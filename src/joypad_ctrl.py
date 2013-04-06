#!/usr/bin/env python

'''************************************************************************************************************************
This node offers joypad-control to an attached ardrone (using ardrone_autonomy)
It is independent from other nodes of this project
***************************************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
************************************************************************************************************************'''


# system
import os, sys
# ros
import roslib;
roslib.load_manifest('yudrone')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import Joy
#local
from singleton import SingletonType

'''************************************************************************************************************************
node for joypad control
************************************************************************************************************************''' 
class joypad_ctrl():
  __metaclass__ = SingletonType
  
  def __init__(self):
    '''
    Constructor 
    '''    
    JOYPAD = rospy.get_param("joy_node/dev")
    joypadConnected = os.system('test -e ' + JOYPAD)
    if joypadConnected != 0:
      rospy.logerr('no joypad connected on ' + JOYPAD)
    else:
      rospy.loginfo('joypad is connected')
    
    joyNodeRunning = os.system('rosnode list | grep /joy_node > /dev/null')
    if joyNodeRunning != 0:
      rospy.logerr('joynode is not running')
    else:
      rospy.loginfo('joynode is running')
    
    rospy.init_node('yudrone_joy')
    self.hasControl = True
    
    #publishers
    self.pub_land = rospy.Publisher( "ardrone/land", Empty )
    self.pub_takeoff = rospy.Publisher( "ardrone/takeoff", Empty )
    self.pub_emergency = rospy.Publisher( "ardrone/reset", Empty )
    self.pub_yaw = rospy.Publisher( "/cmd_vel", Twist )
    
    #subscribers
    self.sub_joy = rospy.Subscriber( "joy", Joy, self.handle_joy )
    self.sub_twist = rospy.Subscriber( "yudrone/cmd_vel", Twist, self.handle_twist )
    rospy.sleep(0.1)
    
    rospy.loginfo('joypad_ctrl initialized')
  
  def handle_twist(self, twist):
    '''
    Callback function for incomming twist msgs
    will be forwarded to ardrone only if gamepad does not have exclusive control
    '''
    if self.hasControl == False:
      self.pub_yaw.publish(twist)
    else:
      rospy.logwarn('ardrone controlled by gamepad: twist was not sent')
      
  def handle_joy(self, joy):
    '''
    Callback function for incomming joypad commands
    '''
    # btn nr 4 for takeoff
    if joy.buttons[3]==1:
      rospy.loginfo('start command')
      self.pub_takeoff.publish( Empty() )
      rospy.sleep(0.1)
    # btn nr 2 for land
    elif joy.buttons[1]==1:
      rospy.loginfo('land command')
      self.pub_land.publish( Empty() )
      rospy.sleep(0.1)
    # btn nr 1 for emergency
    elif joy.buttons[0]==1:
      rospy.loginfo('emergency mode toggled')
      self.pub_emergency.publish( Empty() )
      rospy.sleep(0.1)
    # btn nr 3 for stop
    elif joy.buttons[2]==1:
      if self.hasControl == False:
	self.hasControl = True
	yaw = Twist()
	self.pub_yaw.publish(yaw)
	rospy.loginfo('gamepad has full control')
      else:
	self.hasControl = False
	rospy.loginfo('twist is conntrolled by commands')
	
      
    # joysticks  
    elif self.hasControl == True:
      # set yaw parameter
      yaw = Twist()
      yaw.angular.x = yaw.angular.y = 0
      yaw.angular.z = joy.axes[2] * 3.14/2
      yaw.linear.z = joy.axes[3] * 2.0
      yaw.linear.x = joy.axes[1] * 1.0
      yaw.linear.y = joy.axes[0] * 1.0
    
      # publish yaw to ardrone
      self.pub_yaw.publish(yaw)
    else:
      rospy.loginfo('gamepad locked')
      
'''************************************************************************************************************************
top-level-code
************************************************************************************************************************'''
if __name__ == '__main__':
  try:
    # start node
    ctrl = joypad_ctrl()
    rospy.spin()
  except:
    print 'Exception occured'
    for line in sys.exc_info():
      print(line)