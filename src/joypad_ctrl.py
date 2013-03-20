#!/usr/bin/env python

'''************************************************************************************************************************
This node offers joypad-control to an attached ardrone
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
main class of project (controller & view)
************************************************************************************************************************''' 
class joypad_ctrl():
  __metaclass__ = SingletonType
  
  def __init__(self):
    '''
    Constructor initializes
    * joy_node
    * own node
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
    
    rospy.init_node('joypad_ctrl')
    self.lock = False
    
    #publishers
    self.pub_land = rospy.Publisher( "ardrone/land", Empty )
    self.pub_takeoff = rospy.Publisher( "ardrone/takeoff", Empty )
    self.pub_emergency = rospy.Publisher( "ardrone/reset", Empty )
    self.pub_yaw = rospy.Publisher( "/cmd_vel", Twist )
    
    #subscribers
    self.sub_joy = rospy.Subscriber( "joy", Joy, self.handle_joy )
    self.sub_lock = rospy.Subscriber( "/yudrone/lock_joypad", Bool, self.handle_lock )
    rospy.sleep(0.1)
    
    rospy.loginfo('joypad_ctrl initialized')
    
  def handle_lock(self, msg):
    if msg.data == True:
      self.lock = True
      rospy.loginfo('gamepad locked')
    else:
      self.lock = False
      rospy.loginfo('gamepad unlocked')
    
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
    if joy.buttons[1]==1:
      rospy.loginfo('land command')
      self.pub_land.publish( Empty() )
      rospy.sleep(0.1)
    # btn nr 1 for emergency
    if joy.buttons[0]==1:
      rospy.loginfo('emergency mode toggled')
      self.pub_emergency.publish( Empty() )
      rospy.sleep(0.1)
    # btn nr 3 for stop
    if joy.buttons[2]==1:
      yaw = Twist()
      rospy.loginfo('stop twist')
      self.pub_yaw.publish(yaw)
      # TODO add suificcient stop for commands
      
    # joysticks  
    if self.lock == False:
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