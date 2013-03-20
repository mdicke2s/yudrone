'''***********************************************************************************************
This file contains all commands used from the command line interface.
A python console is embedded in the yudrone gui, that performs these commands.
**************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
***********************************************************************************************'''

from flight import *
from ar_recog.msg import Tags, Tag
from geometry_msgs.msg import Twist
import rospy
import math
roslib.load_manifest('yudrone')

PATTERNS = {0:'4x4_45.patt', 1:'4x4_47.patt', 2:'4x4_91.patt', 3:'4x4_93.patt', 4:'patt.hiro'}
SEARCH = {0:10, 1:-30, 2:60, 3:-100, 4:150, 5:-200, 6:360}
NAVRATE = 10.0 #Hz

class Commands:
  '''****************************************************************************************************
  *										administrative methods
  ****************************************************************************************************'''
  '''
  constructor
  (optional parameter) flight is yudrone gui
  '''
  def __init__(self, flight=None):
    
    self.__facedTagNr = -1
    self.__maxAltitude = 2000
    self.__minAltitude = 100
    self.__yawSpeed = 100
    self.__hrzSpeed = 100
    self.__searchState = 0
    self.__aim = {'ax':0.0, 'ay':0.0, 'az':0.0, 'lx':0.0, 'ly':0.0, 'lz':0.0}
    
    if flight == None:
      # this code applies for usage WITHOUT gui
      rospy.init_node('yudrone_cmd')
      rospy.loginfo("init 'yudrone commands' stand-alone")
      rospy.loginfo("make sure the following nodes are running: ardrone_driver, ar_pose")
      self.updateNavSwitch(True)
      self.hasGui = False
    else:
      # this code applies for usage WITH gui
      rospy.loginfo("init 'yudrone commands' using gui")
      self.updateNavSwitch(False)
      self.hasGui = True

    self.pub_land = rospy.Publisher( "ardrone/land", Empty )
    self.pub_takeoff = rospy.Publisher( "ardrone/takeoff", Empty )
    self.pub_emergency = rospy.Publisher( "ardrone/reset", Empty )
    self.pub_yaw = rospy.Publisher( "/cmd_vel", Twist )
    self.sub_nav = rospy.Subscriber( "ardrone/navdata", Navdata, self.updateNav )
    self.sub_tags = rospy.Subscriber( "tags", Tags, self.updateTags )
    
    self.__reset_twist(0.1)
    self.navTimer = rospy.Timer(rospy.Duration(1.0/NAVRATE), self.__onNavigate)
    self.navdata = None
  
  '''
  this function is contigiously called by a timer  
  rate = NAVRATE
  '''
  def __onNavigate(self, event = None):
    if self.NavigateSwitch == True:
      # caculate values
      yaw = Twist()
      yaw.angular.x = self.__aim['ax']
      yaw.angular.y = self.__aim['ay']
      yaw.angular.z = self.__aim['az']
      # p-controller
      #if self.navdata.altd > 50:
	#yaw.linear.z = (self.__aim['lz'] - self.navdata.altd) /10
	#rospy.loginfo('lz: ' + str(yaw.linear.z))
      yaw.linear.z = self.__aim['lz']
      yaw.linear.x = self.__aim['lx']
      yaw.linear.y = self.__aim['ly']

      # publish yaw to ardrone
      self.pub_yaw.publish(yaw)
    
  '''
  set navdata (from ardrone)
  '''
  def updateNav(self, navdata):
    self.navdata = navdata
    
  '''
  set tags (from artoolkit)
  '''
  def updateTags(self, tagMsg):
    self.tagMsg = tagMsg
  
  '''
  change camera resolution
  '''
  def updateResolution(self, height, width):
    rospy.loginfo('camera resolution set to %d x %d' %(width, height))
    self.imgheight = height
    self.imgwidth = width
    
  '''
  enable or disable __onNavigate
  '''
  def updateNavSwitch(self, switch):
    self.NavigateSwitch = switch
    if switch == True:
      rospy.loginfo('command driven navigation ENABLED')
      self.__reset_twist()
    else:
      rospy.loginfo('command driven navigation DISABLED')
      self.__reset_twist()
    
  
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
    
    # set designated altitude
    self.__aimedAltd = self.navdata.altd + delta
    if self.__aimedAltd < self.__minAltitude:
      self.__aimedAltd = self.__minAltitude
    elif self.__aimedAltd > self.__maxAltitude:
      self.__aimedAltd = self.__maxAltitude
    self.__altdDelta = delta
    
    # lift ardrone by until in range of designated altitude
    self.__aim['lz'] = delta
    reading1 = self.navdata.altd
    rospy.loginfo('current altitude = %i \t aimed altitude = %i' %(reading1, self.__aimedAltd))    
    
    # stop callback
    rospy.Timer(rospy.Duration(0.1), self.__altdStop, oneshot=True)
  
  '''
  this function is a callback to stop altitude changing after the designated value was reached
  '''
  def __altdStop(self, event=None):
    # get two altitude readings
    reading1 = self.navdata.altd
    rospy.sleep(0.1)
    reading2 = self.navdata.altd
    rospy.loginfo('altitude %i'%reading1)
    
    if ( (self.__altdDelta > 0 and reading1 > self.__aimedAltd) or # upwards
      (self.__altdDelta <= 0 and reading1 < self.__aimedAltd) or # downwards
        reading1 <= self.__minAltitude or # reached bottom
        reading1 >= self.__maxAltitude ): # reached top
      #stop lifting
      rospy.loginfo('reached altitude @ %i'%reading1)
      self.__reset_twist(0)
    else:
      #call again
      rospy.Timer(rospy.Duration(0.1), self.__altdStop, oneshot=True) 
    
  def MaxAltd(self, val):
    '''
    Action:
      Sets threshold value. Once defined, thefunction will not exceed those values any more.
      If not defined, the maximal altitude is limited by a firmware defined threshold.
    Parameter:
      val [mm]
    '''
    print('MaxAltd set to ' + str(val))
    self.__maxAltitude = val
    
  def MinAltd(self, val):
    '''
    Action:
      Sets threshold values. Once defined, thefunction will not exceed those values any more.
      If not defined, the maximal altitude is limited by the ground.
    Parameter:
      val [mm]
    '''
    print('MinAltd set to ' + str(val))
    self.__minAltitude = val
    
  '''****************************************************************************************************
  *											basic navigation
  ****************************************************************************************************'''
  def __reset_twist(self, delay = 0):
    '''
    pushes a zero to yaw aim
    '''
    if delay == 0:
      self.__onReset()
    else:
      self.__reset_twist_timer = rospy.Timer(rospy.Duration(delay), self.__onReset, oneshot=True)
    
  def __onReset(self, event=None, silent = False):
    self.__aim['ax'] = self.__aim['ay'] = self.__aim['az'] = 0
    self.__aim['lz'] = self.__aim['lx'] = self.__aim['ly'] = 0
    if silent == True:
      rospy.loginfo('Aim=(0,0,0,0,0,0)')
  
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
    self.__reset_twist(math.fabs(angle)/100)
    
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
    self.__reset_twist(vectLen/100)
    
  def HrzSpeed(self, val):
    '''
    Action:
      Sets the speed to be used performing Horizontal().
    Parameter:
      1, 2, ..., 255
    '''
    print('HrzSpeed set to ' + str(val))
    self.__hrzSpeed = val
    
  def TakeOff(self):
    '''
    Action:
      Triggers built-in takeoff command.    
    '''
    print('TakeOff cmd')
    self.pub_takeoff.publish( Empty() )
    
  def Land(self):
    '''
    Action:
      Triggers built-in land command.    
    '''
    print('Land cmd')
    self.pub_land.publish( Empty() )
  
  def ToggleEmerg(self):
    '''
    Action:
      Toggles the emergency mode of Ardrone. Red LEDs indicate emergency mode, where all
      rotors are stopped. Without emergency these LEDs show up green.
    '''
    print('Emegr cmd')
    self.pub_emergency.publish( Empty() )
    
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
  def Face(self, tagNr, faceYaw = True, faceDist = True, faceAlt = True, facePerpend = True, keepSearching = False):
    '''
    Action:
      Faces a specified tag. As long as this tag is visible in the field of view, the Ardrone will
      automatically yaw towards the target. This function overrides yaw control, whereas
      horizontal motion is not affected. Whenever the system loses the target, it will prompt a warning.
    Parameter:
      tagNr (integer)
    '''
    print('Faceing "' + PATTERNS[tagNr] + '"')
    self.__facedTagNr = tagNr
    self.__faceLostCounter = 0
    
    self.__faceKeepSearching = keepSearching
    self.__faceYaw = faceYaw
    self.__facePerpend = facePerpend
    self.__faceDist = faceDist
    self.__faceAlt = faceAlt
    
    # start facing
    self.__onFace()
  
  def __onFace(self, event=None):              
    # select tag
    facedTag = Tag()
    for tag in self.tagMsg.tags:
      if (tag.id == self.__facedTagNr):
        facedTag = tag

    if facedTag.diameter == 0:
      # designated tag is not in range
      self.__onReset() # stop moving
      if self.__reset_twist_timer.isAlive():
        # stop last running timer
        self.__reset_twist_timer.shutdown()
      
      # start search
      if self.__faceKeepSearching == True:
        rospy.Timer(rospy.Duration(0.1), self.__onSearch, oneshot=True)
      
      self.__faceLostCounter += 1
      if self.__faceLostCounter % 10 == 0:	# 1Hz
        rospy.logwarn('tag not in range')
      if self.__faceLostCounter > 200:		# after 20 seconds
        rospy.logerr('tag is lost')
        self.Release()
    
    else:
      # face
      if self.__faceLostCounter > 3: # got lost tag back
        rospy.loginfo('tag in range')
        self.__searchState = -1# reset search state
        self.__onSearch()	# will stop search. because stage=-1
      self.__faceLostCounter = 0
      
      cx = 0; cy = 0
      for i in [0,2,4,6]:
        cx = cx + facedTag.cwCorners[i]
        cy = cy + facedTag.cwCorners[i+1]
      cx = cx / 4. / self.imgwidth
      cy = cy / 4. / self.imgheight
      
      # rotate to face the tag
      if self.__faceYaw == True:
        self.__aim['az'] = (-(cx - .5)/.5)
      
      # elevate to face tag
      if self.__faceAlt == True:
        self.__aim['lz'] = (-(cy - .5)/.5)
      
      # move to certain distance of tag
      if self.__faceDist == True:
        deltaDist = facedTag.distance - 1200;	# designated to be 1.2 m from target
      if abs(deltaDist) > 50:
        self.__aim['lx'] = 0.05 * deltaDist/abs(deltaDist)
        if abs(deltaDist) > 300:
          self.__aim['lx'] = 0.1 * deltaDist/abs(deltaDist)
      else:
        self.__aim['lx'] = 0
      
      # move perpendicular to tag
      if self.__facePerpend == True:
        self.__aim['ly'] = 0.25 * facedTag.yRot
      
    
    # keep faceing
    if self.__facedTagNr > -1:
      rospy.Timer(rospy.Duration(0.1), self.__onFace, oneshot=True)      
  
  
  def Release(self):
    '''
    Action:
      stops faceing of tag (specified by Face(tagNr))
    Parameter:
      tagNr (integer)
    '''
    if self.__facedTagNr > -1:
      print('released tag nr %i', self.__facedTagNr)
      self.__facedTagNr = -1
    
  def Search(self, tagNr):
    '''
    Action:
      Ardrone will yaw slowly, until the specified tag [direction (string)] appears. Overrides all movement controls.
      If it rotated full 360 without finding the target, a warning is prompted.
      [The optional parameter direction may be set "-",changing the direction to "clockwise"]
    Parameter:
      tagNr (integer)
    '''
    print('Searching ' + str(tagNr))
    self.__searchState = 0
    # todo self.___searchTag???
    rospy.Timer(rospy.Duration(0.1), self.__onSearch, oneshot=True)
    
  def __onSearch(self, event=None):
    if self.__faceLostCounter < 3 or self.__searchState == -1:
      self.__reset_twist(0) #cancel search
      self.__searchState = 0 # reset search parameter
    else:	# lost since 0.4s
      # keep searching
      if self.__aim['az'] == 0: # only if last yaw finished # TODO wait until duration is over
        rospy.loginfo('performing search stage %i'%self.__searchState)
      if self.__reset_twist_timer.isAlive():
        self.__reset_twist_timer.shutdown()
      self.Yaw(SEARCH[self.__searchState])
      duration = rospy.Duration(math.fabs(self.__searchState)/100) 
      self.__searchState = (self.__searchState + 1) % 7
      # call again
      rospy.Timer(rospy.duration(0.1), self.__onSearch, oneshot=True)
      
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
  
'''************************************************************************************************************************
top-level-code
************************************************************************************************************************'''
if __name__ == '__main__':
  try:
    # start node
    cmdNode = Commands()
    rospy.spin()
  except:
    print 'Exception occured'
    for line in sys.exc_info():
      print(line)