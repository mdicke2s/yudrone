'''***********************************************************************************************
This file contains all commands used from the command line interface.
A python console is embedded in the yudrone gui, that performs these commands.
**************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
***********************************************************************************************'''

from flight import *
from geometry_msgs.msg import Twist
import rospy
import math



class Commands:
  '''****************************************************************************************************
  *										administrative methods
  ****************************************************************************************************'''
  def __init__(self, flight):
    self.flight = flight
    self.maxAltitude = 3000
    self.minAltitude = 50
    self.yawSpeed = 100
    self.hrzSpeed = 100
    self.aim = {'ax':0.0, 'ay':0.0, 'az':0.0, 'lx':0.0, 'ly':0.0, 'lz':0.0}
    
    self.navTimer = rospy.Timer(rospy.Duration(0.1), self.__onNavigate)
    self.navdata = None

  def __onNavigate(self, event = None):
    if self.flight.rbJoypad.GetValue() == False:
      # caculate values
      yaw = Twist()
      yaw.angular.x = self.aim['ax']
      yaw.angular.y = self.aim['ay']
      yaw.angular.z = self.aim['az']
      # p-controller
      #if self.navdata.altd > 50:
	#yaw.linear.z = (self.aim['lz'] - self.navdata.altd) /10
	#self.flight.pub_log.publish('lz: ' + str(yaw.linear.z))
      yaw.linear.z = self.aim['lz']
      yaw.linear.x = self.aim['lx']
      yaw.linear.y = self.aim['ly']

      # publish yaw to ardrone
      self.flight.pub_yaw.publish(yaw)
    
  def updateNav(self, navdata):
    self.navdata = navdata
    
  def updateTag(self, tag):
    self.tag = tag
  
  '''****************************************************************************************************
  *												tresholds
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
    print('Altitude set to ' + str(delta))
    # set yaw parameter
    self.aim['lz'] = delta
    self.__reset_Yaw(math.fabs(delta)/200)
    
  def MaxAlt(self, val):
    '''
    Action:
      Sets threshold value. Once defined, thefunction will not exceed those values any more.
      If not defined, the maximal altitude is limited by a firmware defined threshold.
    Parameter:
      val [mm]
    '''
    print('MaxAlt set to ' + str(val))
    self.maxAltitude = val
    
  def MinAlt(self, val):
    '''
    Action:
      Sets threshold values. Once defined, thefunction will not exceed those values any more.
      If not defined, the maximal altitude is limited by the ground.
    Parameter:
      val [mm]
    '''
    print('MinAlt set to ' + str(val))
    self.minAltitude = val
    
  '''****************************************************************************************************
  *											basic navigation
  ****************************************************************************************************'''
  def __reset_Yaw(self, delay = 0):
    '''
    pushes a zero to yaw aim
    '''
    self.flight.txtAim.SetLabel('Aim:\t\t\t\t' + str(self.aim))
    rospy.Timer(rospy.Duration(delay), self.__onReset, oneshot=True)
    
  def __onReset(self, event):
    self.aim['ax'] = self.aim['ay'] = self.aim['az'] = 0
    self.aim['lz'] = self.aim['lx'] = self.aim['ly'] = 0
    self.flight.txtAim.SetLabel('Aim:\t\t\t\t' + str(self.aim))
  
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
    self.aim['az'] = angle/math.fabs(angle) * self.yawSpeed
    self.__reset_Yaw(math.fabs(angle)/100)
    
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
    self.flight.pub_log.publish('vectlen' + str( vectLen))
    self.aim['lx'] = x/vectLen * self.hrzSpeed
    self.aim['ly'] = y/vectLen * self.hrzSpeed
    self.flight.pub_log.publish('lx: ' + str(self.aim['lx']))
    self.__reset_Yaw(vectLen/100)
    
  def HrzSpeed(self, val):
    '''
    Action:
      Sets the speed to be used performing Horizontal().
    Parameter:
      1, 2, ..., 255
    '''
    print('HrzSpeed set to ' + str(val))
    self.hrzSpeed = val
    
  def TakeOff(self):
    '''
    Action:
      Triggers built-in takeoff command.    
    '''
    print('TakeOff cmd')
    self.flight.pub_takeoff.publish( Empty() )
    
  def Land(self):
    '''
    Action:
      Triggers built-in land command.    
    '''
    print('Land cmd')
    self.flight.pub_land.publish( Empty() )
  
  def ToggleEmerg(self):
    '''
    Action:
      Toggles the emergency mode of Ardrone. Red LEDs indicate emergency mode, where all
      rotors are stopped. Without emergency these LEDs show up green.
    '''
    print('Emegr cmd')
    self.flight.pub_emergency.publish( Empty() )
    
  '''****************************************************************************************************
  *												Batch
  ****************************************************************************************************'''
  def Pause(self):
    '''
    Action:
      Interrupts the current command. Any incoming command will be ignored during pause.
    '''
    print('Pause cmd' + ' __unimplemented')
    
  def Continue(self):
    '''
    Action:
      Continues execution of current command and exits pause mode.
    '''
    print('Continue cmd' + ' __unimplemented')
    
  def Break(self):
    '''
    Action:
      Same as pause, but additionally the current action will be dismissed.   
    '''
    print('Break cmd' + ' __unimplemented')
    
  def Batch(self, fileName):
    '''
    Action:
      Runs the specified batch-file.
    Parameter:
      fileName (string)
    '''
    print('Run ' + str(fileName) + ' __unimplemented')  
    
  def Exit(self):
    '''
    Action:
      Cancels a batch run.
    '''
    print('Cancel cmd' + ' __unimplemented')
    
  '''****************************************************************************************************
  *											high level navigation
  ****************************************************************************************************'''
  def Face(self, tagNr):
    '''
    Action:
      Faces a specified tag. As long as this tag is visible in the field of view, the Ardrone will
      automatically yaw towards the target. This function overrides yaw control, whereas
      horizontal motion is not affected. Whenever the system loses the target, it will prompt a warning.
    Parameter:
      tagNr (integer)
    '''
    print('Face cmd' + ' __unimplemented')
    
  def Release(self):
    '''
    Action:
      Releases facing tag specified by Face().
    '''
    print('release cmd' + ' __unimplemented')
    
  def Search(self, tagNr):
    '''
    Action:
      Ardrone will yaw slowly, until the specified tag [direction (string)] appears. Overrides all movement controls.
      If it rotated full 360 without finding the target, a warning is prompted.
      [The optional parameter direction may be set "-",changing the direction to "clockwise"]
    Parameter:
      tagNr (integer)
    '''
    print('Searching ' + str(tagNr) + ' __unimplemented')
    
  def Approach(self, tagNr):
    '''
    Action:
      Ardrone will automatically search and approach the specified tag as described in 3.2.
      Overrides all movement controls.
    Parameter:
      tagNr (integer)
    '''
    print('Approaching ' + str(tagNr) + ' __unimplemented') 
    
  def Navigate(self, x, y, z, xRot, yRot, zRot):
    '''
    Action:
      Performs a graph-based navigation as described in 3.3. Overrides all movement controls.
      Thisfunction it will require an initialization of the graph environment.
    Parameter:
      x, y, z,xRot, yRot, zRot (all integer) 
    '''
    print('Navigate cmd' + ' __unimplemented') 
  
