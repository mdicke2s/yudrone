#!/usr/bin/env python

'''***********************************************************************************************
This node provides abstracted access to ardrone quadrotor
Basically it encapsulates all communication to control the vehicles 3d-motion:
 * altitude (Z-axis)
 * yaw and horizontal motion control (X-Y-plane)
Therefore it is using /cmd_vel other nodes shouldn't publish to this topic at the same time
**************************************************************************************************
Project: yudrone
Author: Michael Dicke
Repository: https://github.com/mdicke2s/yudrone
***********************************************************************************************'''


# OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE 
# OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE # OBSOLETE 


#from geometry_msgs.msg import Twist
#import rospy
#import math
#import roslib;
#roslib.load_manifest('yudrone')
#from ardrone_autonomy.msg import Navdata

NAVRATE = 10.0 #Hz

class yudrone_delegate:
  '''****************************************************************************************************
  * administrative methods
  ****************************************************************************************************'''
  
  def __init__(self):
    '''
    constructor
    '''
    rospy.init_node('yudrone_delegate')
    rospy.loginfo("init 'yudrone_delegate'")
    #self.pub_emergency = rospy.Publisher( "ardrone/reset", Empty ) # will be necessary if adding watchdog
    self.pub_twist = rospy.Publisher( "/cmd_vel", Twist )
    self.sub_nav = rospy.Subscriber( "ardrone/navdata", Navdata, self.updateNav )
    self.updateNavSwitch(True)
      
    self.__maxAltitude = 3000  # ~ lab ceiling
    self.__minAltitude = 50	# ~ 5cm over ground
    self.__yawSpeed = 100
    self.__hrzSpeed = 100
    self.__aim = {'ax':0.0, 'ay':0.0, 'az':0.0, 'lx':0.0, 'ly':0.0, 'lz':0.0}
    
    self.__reset_aim(0.1)
    self.navTimer = rospy.Timer(rospy.Duration(1.0/NAVRATE), self.__onNavigate)
    self.__navdata = None
  
  def __onNavigate(self, event = None):
    '''
    this function is contigiously called by a timer
    rate = NAVRATE
    '''
    if self.NavigateSwitch == True:
      # assemble twist message
      twist = Twist()
      twist.angular.x = self.__aim['ax']
      twist.angular.y = self.__aim['ay']
      twist.angular.z = self.__aim['az']
      twist.linear.z = self.__aim['lz']
      twist.linear.x = self.__aim['lx']
      twist.linear.y = self.__aim['ly']
  
      # publish twist to ardrone
      self.pub_twist.publish(twist)
  
  def updateNav(self, navdata):
    '''
    set navdata (comming from ardrone)
    '''
    self.__navdata = navdata
	
  def updateNavSwitch(self, switch):
    '''
    enable or disable __onNavigate
    '''
    self.NavigateSwitch = switch
    if switch == True:
      rospy.loginfo('delegate driven navigation ENABLED')
    else:
      rospy.loginfo('delegate driven navigation DISABLED')
    
    
  '''****************************************************************************************************
  * tresholds
  ****************************************************************************************************'''
	
  def Altitude(self, delta):
    '''
    Action:
    In-/decreases the UAVs hovering altitude by a certain value.
    The function uses the scale given by the IMUs altimeter.
    Parameter:
    delta [mm] (integer)
    positive means up
    '''    
    # set designated altitude
    self.__aimedAltd = self.__navdata.altd + delta
    if self.__aimedAltd < self.__minAltitude:
      self.__aimedAltd = self.__minAltitude
    elif self.__aimedAltd > self.__maxAltitude:
      self.__aimedAltd = self.__maxAltitude
    self.__altdDelta = delta
    
    # lift ardrone by until in range of designated altitude
    self.__aim['lz'] = delta
    reading1 = self.__navdata.altd
    rospy.loginfo('current altitude = %i \t aimed altitude = %i' %(reading1, self.__aimedAltd))
    
    # stop callback
    rospy.Timer(rospy.Duration(0.1), self.__altdStop, oneshot=True)
  
  def __altdStop(self, event=None):
    '''
    this function is a callback to stop altitude changing after the designated value was reached
    '''
    # get two altitude readings
    reading1 = self.__navdata.altd
    rospy.sleep(0.1)
    reading2 = self.__navdata.altd
    rospy.loginfo('altitude %i'%reading1)
    
    if ( (self.__altdDelta > 0 and reading1 > self.__aimedAltd) or # upwards
         (self.__altdDelta <= 0 and reading1 < self.__aimedAltd) or # downwards
         reading1 <= self.__minAltitude or # reached bottom
         reading1 >= self.__maxAltitude ): # reached top
		 
      #stop lifting
      rospy.loginfo('reached altitude @ %i'%reading1)
      self.__reset_aim(0)
    else:
      #call again
      rospy.Timer(rospy.Duration(0.1), self.__altdStop, oneshot=True) 
      
  def MaxAlt(self, val):
    '''
    Action:
    Sets threshold value. Once defined, thefunction will not exceed those values any more.
    If not defined, the maximal altitude is limited by a firmware defined threshold.
    Parameter:
    val [mm]
    '''
    print('MaxAlt set to ' + str(val))
    self.__maxAltitude = val
      
  def MinAlt(self, val):
    '''
    Action:
    Sets threshold values. Once defined, thefunction will not exceed those values any more.
    If not defined, the maximal altitude is limited by the ground.
    Parameter:
    val [mm]
    '''
    print('MinAlt set to ' + str(val))
    self.__minAltitude = val
      
  '''****************************************************************************************************
  * navigation
  ****************************************************************************************************'''
  def __reset_aim(self, delay = 0):
    '''
    pushes a zero to aim
    '''
    if delay == 0:
      self.__onReset()
    else:
      rospy.Timer(rospy.Duration(delay), self.__onReset, oneshot=True)
      
  def __onReset(self, event = None):
    self.__aim['ax'] = self.__aim['ay'] = self.__aim['az'] = 0
    self.__aim['lz'] = self.__aim['lx'] = self.__aim['ly'] = 0
  
  def Yaw(self, angle):
    '''
    Action:
    Rotates the UAV by a specified angle (yaw).
    The device will turn slowly until the designated angle is reached.
    Therefore it accesses the rotation angle provided by the internal compass.
    Parameter:
    angle [deg] (integer) -180, ..., +180 negative is clockwise
    '''
    print('Yaw ' + str(angle))
    # set yaw parameter
    self.__aim['az'] = angle/math.fabs(angle) * self.__yawSpeed
    self.__onCmdOut()
    self.__reset_aim(math.fabs(angle)/100)
      
  def YawSpeed(self, val):
    '''
    Action:
    Sets the speed to be used performing Yaw().
    Parameter:
    val = 1, 2, ..., 255
    '''
    print('YawSpeed set to ' + str(val) + ' __unimplemented')
      
  def Horizontal(self, x, y):
    '''
    Action:
    Moves the UAV a certain distance in a specified horizontal direction (see figure 1).
    The distance cannot be determined exactly, but it is approximated by experienced data
    and the horizontal speed.
    Parameter:
    x[cm], y[cm] (both integer)
    '''
    print('Horizontal movement')
    # set yaw parameter
    vectLen = math.sqrt( x*x + y*y )
    rospy.loginfo('vectlen' + str( vectLen))
    self.__aim['lx'] = x/vectLen * self.__hrzSpeed
    self.__aim['ly'] = y/vectLen * self.__hrzSpeed
    rospy.loginfo('lx: ' + str(self.__aim['lx']))
    self.__reset_aim(vectLen/100)
      
  def HrzSpeed(self, val):
    '''
    Action:
    Sets the speed to be used performing Horizontal().
    Parameter:
    1, 2, ..., 255
    '''
    print('HrzSpeed set to ' + str(val))
    self.__hrzSpeed = val
    
'''************************************************************************************************************************
top-level-code
STARTS application
************************************************************************************************************************'''
if __name__ == '__main__':
  try:
    # start node
    yudrone_delegate()
    
  except:
    print 'Exception occured'
    for line in sys.exc_info():
      print(line)
