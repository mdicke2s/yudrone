#!/usr/bin/env python

'''************************************************************************************************************************
This node monitors ardrones status and forces emergency mode if something is wrong
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
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
from ar_recog.msg import Tags
#local
from singleton import SingletonType

'''************************************************************************************************************************
watchdog node
************************************************************************************************************************''' 
class watchdog():
  __metaclass__ = SingletonType
  
  def __init__(self):
    '''
    Constructor
    '''
    
    rospy.init_node('yudrone_watchdog')
    
    # check ar_node
    self.arNodeRunning = os.system('rostopic list | grep /ar/image > /dev/null')
    if self.arNodeRunning != 0:
      rospy.logerr('ar_node is not running')
      self.__imageTopic = '/ardrone/image_raw'
    else:
      rospy.loginfo('ar_node is running')
      self.__imageTopic = '/ar/image'
    
    #publishers
    self.pub_emergency = rospy.Publisher( "ardrone/reset", Empty )
    self.pub_land = rospy.Publisher( "ardrone/land", Empty )
    self.pub_yaw = rospy.Publisher( "/cmd_vel", Twist )
    if self.arNodeRunning == 0:
      self.sub_tags = rospy.Subscriber( "tags", Tags, self.handle_tags )
    
    #subscribers
    self.sub_nav = rospy.Subscriber( "ardrone/navdata", Navdata, self.handle_nav )
    self.sub_image = rospy.Subscriber(self.__imageTopic, Image,self.handle_image)
    rospy.sleep(0.1)
    
    self.watchdog = rospy.Timer(rospy.Duration(2.0), self.handle_watchdog)
    self.lastTags = rospy.get_rostime()
    self.lastImage = rospy.get_rostime()
    self.lastNavdata = rospy.get_rostime()
    self.shutdownCnt = 0
    self.isProcessingShutdown = False
    rospy.loginfo('watchdog initialized')
    
  def handle_nav(self, msg):
    self.lastNavdata = rospy.get_rostime()
    
  def handle_image(self, msg):
    self.lastImage = rospy.get_rostime()
    
  def handle_tags(self, msg):
    self.lastTags = rospy.get_rostime()
    
  def handle_watchdog(self, event=None):
    if not rospy.is_shutdown():
      ardroneDriverRunnig = os.system('rosnode ping -c2 /ardrone_driver | grep "xmlrpc reply from" > /dev/null')
      if ardroneDriverRunnig != 0:
	rospy.logerr('NO CONNECTION TO ARDRONE DRIVER')
	print('\a')
      now = rospy.get_rostime()
      
      hasTimeout = False
      if now.secs - self.lastImage.secs > 9:
	rospy.logerr('Image Timeout')
	hasTimeout = True
      elif self.arNodeRunning == 0 and now.secs - self.lastTags.secs > 9:
	rospy.logerr('Tags Timeout')
	hasTimeout = True
      elif now.secs - self.lastNavdata.secs > 9:
	rospy.logerr('Navdata Timeout')
	hasTimeout = True
	
      if hasTimeout == True: 
	self.shutdown()
	  
    
    
  def shutdown(self):
    if not rospy.is_shutdown():
      print('\a')
    
      if self.shutdownCnt < 3 and self.isProcessingShutdown == False:
	self.isProcessingShutdown = True
	rospy.logerr('SHUTDOWN ARDRONE BY WATCHDOG')
	yaw = Twist()
	self.pub_yaw.publish(yaw)
	
	self.pub_land.publish( Empty() )
	rospy.sleep(5)
	self.pub_emergency.publish( Empty() )
	rospy.sleep(1)
	self.pub_emergency.publish( Empty() )
	self.shutdownCnt = self.shutdownCnt + 1
	self.isProcessingShutdown = False
    
'''************************************************************************************************************************
top-level-code
************************************************************************************************************************'''
if __name__ == '__main__':
  try:
    # start node
    wd = watchdog()
    rospy.spin()
  except:
    print 'Exception occured'
    for line in sys.exc_info():
      print(line)