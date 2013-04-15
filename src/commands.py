#!/usr/bin/env python

'''***********************************************************************************************
This file contains all commands used from the command line interface.
A python console is embedded in the yudrone gui, that performs these commands.
Commands can be called in different ways
- directly out of python code on a Commands instance
- via python commandline interface after implementing and instanciating Commands
- using ros-msg commandsMsg.msg
The following commands will block if self.__lock != -1
- Altitude(..)
- Yaw(..)
- Horizontal(..)
- Approach(..)
- Search(..)
- Face(..)
- Navigate(..)
**************************************************************************************************
TODO
* yaw on magnetometer
**************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
***********************************************************************************************'''

import rospy
import roslib;
roslib.load_manifest('yudrone')

from flight import *
from PID_controller import PID_controller
from ar_recog.msg import Tags, Tag
from geometry_msgs.msg import Twist
from yudrone.msg import commandsMsg, commandStatus
from std_msgs.msg import Int32

import math

# Names of patterns
PATTERNS = {0:'alpha', 1:'4x4_19', 2:'4x4_23', 3:'4x4_29',4:'4x4_45',5:'beta',5:'4x4_47',7:'4x4_49',8:'4x4_83',9:'4x4_89',10:'gamma',11:'4x4_91',12:'4x4_93',13:'4x4_94',14:'4x4_98',15:'delta',16:'cents',17:'eta',18:'hiro',19:'iota',20:'epsilon',21:'kanji',22:'kappa',23:'theta',24:'zeta'}
# Facing
FACESTATES = {0:'YAW', 1:'ALTD', 2:'PERPEND', 3:'DIST'}
#STATEDURATION = {0:0.5, 1:0.5, 2:1.5, 3:1.0} # change faceState after X seconds
FACEDIST = 1500 # designated to be 1.5 m far from target
FACELOSTTIME = 10 # 10 seconds
FACESTOPTHRESHOLD = 0.15
# searching (degrees of yaw, altitude)
#SEARCH = ((-30,0), (90,0), (0,150), (-90,0), (-90,0), (0,-300), (90,0), (90,0), (90,0), (0,600), (-90,0), (-90,0), (-90,0), (-90,0), (0, -450), (1,0))
SEARCH = ((-30,0), (45,0), (45,0), (45,0), (45,0), (45,0), (45,0), (45,0), (45,0), (1,0))
#SEARCHSTEPS = 16
SEARCHSTEPS = 10
# rates
NAVRATE = 10.0 #Hz
CONTROLLRATE = 10.0 #Hz


class Commands:
  '''****************************************************************************************************
  *										administrative methods
  ****************************************************************************************************'''

  def __init__(self, flight=None):
    '''
    constructor
    (optional parameter) flight is yudrone gui
    '''
    self.__facedTagNr = -1
    self.__faceLostCounter = 20
    self.__maxAltitude = 1500
    self.__minAltitude = 100
    self.__yawSpeed = 0.8
    self.__hrzSpeed = 0.5
    self.__searchState = -1
    self.__searchTagNr = -1
    self.__lockNr = -1
    self.__lastFaceCorrectionSum = 0.5
    self.imgheight = 360
    self.imgwidth = 640

    self.__aim = {'ax':0.0, 'ay':0.0, 'az':0.0, 'lx':0.0, 'ly':0.0, 'lz':0.0}
    self.navdata = None

    self.pub_land = rospy.Publisher( "ardrone/land", Empty )		# ardrone land command
    self.pub_takeoff = rospy.Publisher( "ardrone/takeoff", Empty )	# ardrone takeoff command
    self.pub_emergency = rospy.Publisher( "ardrone/reset", Empty )	# ardrone emergency mode toggle
    self.sub_nav = rospy.Subscriber( "ardrone/navdata", Navdata, self.updateNav )

    self.sub_tags = rospy.Subscriber( "tags", Tags, self.updateTags )	# artoolkit

    self.pub_cmdStatus = rospy.Publisher( "yudrone/cmdStatus", commandStatus )		# yudrone_cmd reports availability and status
    self.sub_com = rospy.Subscriber( "yudrone/commands", commandsMsg, self.handle_command )	# yudrone_cmd listenes command messages
    self.pub_twist = rospy.Publisher( "yudrone/cmd_vel", Twist )	# forwarded to ardrone navigation topic (except if joypad has control)

    # init PIDs (reference, Kp, Ki, Kd, maxIntegral[=0.25/Ki])
    self.PID_az = PID_controller(0.5, 0.4, 0.005, 1.0, 50)		# yaw to tag controller
    self.PID_lz = PID_controller(0.5, 0.4, 0.001, 1.0, 250)		# elevate to tag controller
    self.PID_ly = PID_controller(0.0, 0.1, 0.0005, 1.5, 500)		# move perpendicular to tag controller
    self.PID_lx = PID_controller(FACEDIST, 0.00005, 0.000005, 0.001, 500)# distance to tag controller

    if flight == None:
      # this code applies for usage WITHOUT gui
      rospy.init_node('yudrone_cmd')
      rospy.loginfo("init 'yudrone commands' stand-alone")
      rospy.loginfo("make sure the following nodes are running: ardrone_driver, ar_recog")
      self.hasGui = False
    else:
      # this code applies for usage WITH gui
      rospy.loginfo("init 'yudrone commands' using gui")
      self.hasGui = True

    # stop moving and start naviagtion loop
    self.__reset_twist(0.1)
    self.navTimer = rospy.Timer(rospy.Duration(1.0/NAVRATE), self.__onNavigate)

  def __onNavigate(self, event = None):
    '''
    this function is contigiously called by a timer
    rate = NAVRATE
    '''

    # caculate values
    twist = Twist()
    twist.angular.x = self.__aim['ax']
    twist.angular.y = self.__aim['ay']
    twist.angular.z = self.__aim['az']
    # p-controller
    #if self.navdata.altd > 50:
      #twist.linear.z = (self.__aim['lz'] - self.navdata.altd) /10
      #rospy.loginfo('lz: ' + str(twist.linear.z))
    twist.linear.z = self.__aim['lz']
    twist.linear.x = self.__aim['lx']
    twist.linear.y = self.__aim['ly']

    # publish twist to ardrone
    self.pub_twist.publish(twist)

  def lockWarning(self):
    '''
    display log warning
    anounce status on cmdStatus topic

    Explanation: The node was locked and is supposed not to react on incomming naviagtion commands
    '''
    rospy.logwarn('Command was not executed, it is locked by (%i)'%self.__lockNr)
    status = commandStatus(id=self.__lockNr, isLocked=True, status="running")
    self.pub_cmdStatus.publish(status)

  def __lock(self, lockNr):
    '''
    sets a lock flag, which blocks other commands to be executed
    usage example:
      from yudrone.msg import commandsMsg
      pub=rospy.Publisher('yudrone_commands', commandsMsg)
      m=commandsMsg()
      m.hasYaw=True
      m.yaw=500
      pub.publish(m)
      # Yaw 500
      pub.publish(m) # after a second
      # Yaw 500
      pub.publish(m) # imediately after last command
      # [WARN] [WallTime: 1363883870.378374] Command was not executed, it is locked by (2)
      print(m.header.seq)
      # 3 <-- this sequential number is set by publishing the message (not by instanciation!)
    '''
    if lockNr > 0:
      if self.__lockNr == -1:
	# if not already locked
	self.__lockNr = lockNr
      status = commandStatus(id=self.__lockNr, isLocked=True, status="running")
      rospy.loginfo('CmdNode is locked by  %i'%self.__lockNr)
      self.pub_cmdStatus.publish(status)
    else:
      rospy.logerr('invalid lockNr: %i'%lockNr)

  def __unlock(self, exitStatus=""):
    '''
    resets lock flag, allowing other commands to be executed

    Parameter exitStatus:
      if not empty, the current executed command is finished with the given status
    '''
    if exitStatus != "":
      # announce exit of old command
      status = commandStatus(id=self.__lockNr, isLocked=False, status=exitStatus)
      self.__lockNr = -1
      rospy.loginfo('Status turned to "%s"'%exitStatus)
      self.pub_cmdStatus.publish(status)

    elif self.__lockNr != -1:
      # dissolve lock and announce availability
      self.__lockNr = -1
      status = commandStatus(id=-1, isLocked=False, status="available")
      rospy.loginfo('Status turned to "available"')
      self.pub_cmdStatus.publish(status)

  def handle_command(self, msg):
    '''
    handle incomming commandsMsg
    '''

    # call command determined by hasXYZ
    if msg.hasTwist == True:
      rospy.loginfo('received twist command')
      self.pub_twist.publish(msg.twist) # or better use funtion???
    if msg.hasAltdDelta == True:
      rospy.loginfo('received altitude command')
      self.Altitude(delta=msg.altdDelta)
      self.__lock(msg.header.seq) # lock instance
    if msg.hasAltdAbs == True:
      rospy.loginfo('received altitude command')
      self.Altitude(absolute=msg.altdAbs)
      self.__lock(msg.header.seq) # lock instance
    if msg.hasMaxAltd == True:
      rospy.loginfo('received max_altitude command')
      self.MaxAltd(msg.MaxAltd)
    if msg.hasMinAltd == True:
      rospy.loginfo('received min_altitude command')
      self.MinAltd(msg.MixAltd)
    if msg.hasYaw == True:
      rospy.loginfo('received yaw command (%i)'%msg.yaw)
      self.Yaw(msg.yaw)
      self.__lock(msg.header.seq) # lock instance
    if msg.hasHorizontal == True:
      rospy.loginfo('received horizontal command %i %i'%(msg.horizontalX, msg.horizontalY))
      self.Horizontal(msg.horizontalX, msg.horizontalY)
      self.__lock(msg.header.seq) # lock instance
    if msg.hasFace == True:
      rospy.loginfo('received face command (%i)'%msg.tagNr)
      self.Face(msg.tagNr)
      self.__lock(msg.header.seq) # lock instance
    if msg.hasRelease == True:
      rospy.loginfo('received release command')
      self.Release()
    if msg.hasSearch == True:
      rospy.loginfo('received search command (%i)'%msg.tagNr)
      self.Search(msg.tagNr, msg.searchStartAltd)
      self.__lock(msg.header.seq) # lock instance
    if msg.hasApproach == True:
      rospy.loginfo('received approach command (%i)'%msg.tagNr)
      self.Approach(msg.tagNr)
      self.__lock(msg.header.seq) # lock instance
    if msg.hasNavigate == True:
      rospy.loginfo('received navigate command')
      self.Navigate(msg.nav_XYZ_RxRyRz[0], msg.nav_XYZ_RxRyRz[1], msg.nav_XYZ_RxRyRz[2], msg.nav_XYZ_RxRyRz[3], msg.nav_XYZ_RxRyRz[4], msg.nav_XYZ_RxRyRz[5])
      self.__lock(msg.header.seq) # lock instance
    if msg.hasYawSpeed == True:
      rospy.loginfo('received yawSpeed command')
      self.YawSpeed(msg.yawSpeed)
    if msg.hasHrzSpeed == True:
      rospy.loginfo('received hrzSpeed command')
      self.HrzSpeed(msg.hrzSpeed)
    if msg.hasTakeoff == True:
      rospy.loginfo('received takeoff command (%i)'%msg.tagNr)
      self.TakeOff(msg.tagNr)
    if msg.hasLand == True:
      rospy.loginfo('received land command (%i)'%msg.tagNr)
      self.Land(msg.tagNr)

    # not implemented by now...
    ## ToggleEmerg(self)
    ## __reset_twist(delay = 0)


  def updateNav(self, navdata):
    '''
    set navdata (from ardrone)
    '''
    self.navdata = navdata

  def updateTags(self, tagMsg):
    '''
    set tags (from artoolkit)
    '''
    self.tagMsg = tagMsg

  def updateResolution(self, height, width):
    '''
    change camera resolution
    '''
    rospy.loginfo('camera resolution set to %d x %d' %(width, height))
    self.imgheight = height
    self.imgwidth = width


  '''****************************************************************************************************
  *												tresholds
  ****************************************************************************************************'''
  def Altitude(self, delta=None, absolute=None, callback=None):
    '''
    Action:
      In-/decreases the UAVs hovering altitude by a certain value.
      The function uses the scale given by the IMUs altimeter.
    Parameter:
      delta [mm] (integer)
      positive means up
    '''
    if self.__lockNr != -1:
      self.lockWarning() # command is not executed, see function description
    else:
      # set designated altitude
      if delta != None:
	self.__aimedAltd = self.navdata.altd + delta
      elif absolute != None:
	self.__aimedAltd = absolute
	delta = self.__aimedAltd - self.navdata.altd
      else:
	rospy.logerr('no altitude specified')
	return
	
      # adjust if altitude exceeds thresholds
      if self.__aimedAltd < self.__minAltitude:
	self.__aimedAltd = self.__minAltitude
      elif self.__aimedAltd > self.__maxAltitude:
	self.__aimedAltd = self.__maxAltitude
      self.__altdDelta = delta

      # lift ardrone by until in range of designated altitude
      self.__aim['lz'] = 0.4 * math.fabs(delta)/delta
      reading1 = self.navdata.altd
      rospy.loginfo('current altitude = %i \t aimed altitude = %i' %(reading1, self.__aimedAltd))

      # stop callback
      if callback == None:
	callback = self.__altdStop # default callback
      rospy.Timer(rospy.Duration(1.0/CONTROLLRATE), callback, oneshot=True)

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
      self.__onResetSilent()
      self.__unlock("done_sucessfully")
    else:
      #call again
      rospy.Timer(rospy.Duration(1.0/CONTROLLRATE), self.__altdStop, oneshot=True)

  def __altdStopSearch(self, event=None):
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
      rospy.Timer(rospy.Duration(1.0/CONTROLLRATE), self.__onSearch, oneshot=True)
      self.__onResetSilent()
    else:
      #call again if tag still lost
      facedTag = self.getTag(self.__searchTagNr)
      if facedTag.diameter != 0:
	self.__onResetSilent()
	self.__searchStepFinished = True
      else:
	rospy.Timer(rospy.Duration(1.0/CONTROLLRATE), self.__altdStopSearch, oneshot=True)

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

  def __onReset(self, event=None):
    '''
    resets twist (= stop moving)
    + loginfo and unlock yudrone/cmdStatus
    '''
    self.__aim['ax'] = self.__aim['ay'] = self.__aim['az'] = 0
    self.__aim['lz'] = self.__aim['lx'] = self.__aim['ly'] = 0
    self.PID_az.reset()
    self.PID_lz.reset()
    self.PID_ly.reset()
    self.PID_lx.reset()
    rospy.loginfo('Aim=(0,0,0,0,0,0)')
    self.__unlock()

  def __onResetSilent(self, event=None):
    '''
    resets twist (= stop moving)
    without loginfo or unlock yudrone/cmdStatus
    '''
    self.__aim['ax'] = self.__aim['ay'] = self.__aim['az'] = 0
    self.__aim['lz'] = self.__aim['lx'] = self.__aim['ly'] = 0
    self.PID_az.reset()
    self.PID_lz.reset()
    self.PID_ly.reset()
    self.PID_lx.reset()
    self.__searchStepFinished = True

  def Yaw(self, angle):
    '''
    Action:
      Rotates the UAV by a specified angle (yaw).
      The device will turn slowly until the designated angle is reached.
      Therefore it accesses the rotation angle provided by the internal compass.
    Parameter:
      angle [deg] (integer) -180, ..., +180 negative is clockwise
    '''
    if self.__lockNr != -1:
      self.lockWarning() # command is not executed, see function description
    else:
      print('Yaw ' + str(angle))
      # set yaw parameter
      self.__aim['az'] = angle/math.fabs(angle) * self.__yawSpeed
      self.__reset_twist(math.fabs(angle)/(100*self.__yawSpeed) )

  def YawSpeed(self, val):
    '''
    Action:
      Sets the speed to be used performing Yaw().
    Parameter:
      val in [0..2]
    '''
    if val > 0.0 and val <=2.0:
      self.__yawSpeed = val
      print('YawSpeed set to ' + str(val))
    else:
      print('invalid YawSpeed ' + str(val))

  def Horizontal(self, x, y):
    '''
    Action:
      Moves the UAV a certain distance in a specified horizontal direction (see figure 1).
      The distance cannot be determined exactly, but it is approximated by experienced data
      and the horizontal speed.
    Parameter:
      x[cm], y[cm] (both integer)
    '''
    if self.__lockNr != -1:
      self.lockWarning() # command is not executed, see function description
    else:
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
      val in [0..1]
    '''
    if val >0.0 and val <=1.0:
      print('HrzSpeed set to ' + str(val))
      self.__hrzSpeed = val
    else:
      print('invalid HrzSpeed ' + str(val))

  def TakeOff(self, tagNr = None):
    '''
    Action:
      Triggers built-in takeoff command.
    '''
    if tagNr == None:
      print('TakeOff cmd')
      self.pub_takeoff.publish( Empty() )
    else:
      print('Starting above tag %i __unimplemented'%tagNr)
      # PID controller on lx and ly (propably different from used ones)
      self.pub_takeoff.publish( Empty() )

  def Land(self, tagNr = None):
    '''
    Action:
      Triggers built-in land command.
    '''
    if tagNr == None:
      print('Land cmd')
      self.pub_land.publish( Empty() )
    else:
      print('Landing on tag %i __unimplemented'%tagNr)
      # PID controller on lx and ly (propably different from used ones)
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
    if self.__lockNr != -1:
      self.lockWarning() # command is not executed, see function description
    else:
      print('Faceing "' + PATTERNS[tagNr] + '"')
      self.__facedTagNr = tagNr
      self.__faceLostCounter = 0

      self.__faceKeepSearching = keepSearching
      self.__faceYaw = faceYaw
      self.__facePerpend = facePerpend
      self.__faceDist = faceDist
      self.__faceAlt = faceAlt
      self.__faceState = 0 # do one step at a time
      self.__faceStateCounter = 0

      # start facing
      self.__onFace()

  def getTag(self, tagNr):
    # select tag
    facedTag = Tag()
    for tag in self.tagMsg.tags:
      if (tag.id == tagNr):
        facedTag = tag
    return facedTag

  def getCentre(self, tag):
    # find tag centre in relation to screen centre
    cx = 0; cy = 0
    for i in [0,2,4,6]:
      cx = cx + tag.cwCorners[i]
      cy = cy + tag.cwCorners[i+1]
    cx = cx / 4. / self.imgwidth
    cy = cy / 4. / self.imgheight
    return {'x':cx, 'y':cy}

  def __onFace(self, event=None):
    '''
    callback for Face(..) [which only initializes facing]
    performs face until 'Release()' or tag lost for longer time
    '''
    facedTag = self.getTag(self.__facedTagNr)

    if facedTag.diameter == 0:
      # designated tag is not in range
      self.__onResetSilent(None) # stop moving (silent)
      if self.__reset_twist_timer.isAlive():
        # stop last running timer
        self.__reset_twist_timer.shutdown()

      # start search
      if self.__faceKeepSearching == True:# and self.__searchState == -1:
	self.Search(self.__facedTagNr)

      self.__faceLostCounter += 1
      if self.__faceLostCounter % int(CONTROLLRATE) == 0:	# 10Hz
        rospy.logwarn('tag not in range')

      if self.__faceLostCounter > CONTROLLRATE*FACELOSTTIME:	# after FACELOSTTIME seconds
        rospy.logerr('tag is lost for %i seconds, faceing canceled'%FACELOSTTIME)
        self.Release()

    else: # designated tag is in range
      # face
      if self.__faceLostCounter > 3: # got lost tag back
        rospy.loginfo('tag in range')
        self.__searchState = -1# reset search state
        self.__faceState = 0   # reset face state
        self.__faceStateCounter = 0
        self.__onSearch()	# will stop search. because stage=-1
        #rospy.loginfo('Face-state = ' + FACESTATES[self.__faceState])
      self.__faceLostCounter = 0

      # find tag centre in relation to screen centre
      centre = self.getCentre(facedTag)
      cx = centre['x']
      cy = centre['y']

      # rotate to face the tag
      if self.__faceYaw == True:# and self.__faceState == 0:
        self.__aim['az'] = self.PID_az.update(cx)

      # elevate to face tag
      if self.__faceAlt == True:# and self.__faceState == 1:
	if self.__maxAltitude > self.navdata.altd and \
	  (self.__minAltitude < self.navdata.altd or \
	  self.navdata.altd == 0): # if altitude is within tresholds or UAV on ground (during dry tests)
	  self.__aim['lz'] = self.PID_lz.update(cy)
	else:
	  self.__aim['lz'] = 0.0

      # move perpendicular to tag [operates on two dimensions]
      if self.__facePerpend == True:# and self.__faceState == 2:
        self.__aim['ly'] = - self.PID_ly.update(facedTag.yRot)
        #self.__aim['az'] = self.PID_az.update(cx)
      else:
        self.__aim['ly'] = 0.0
        #self.__aim['az'] = 0.0

      # move to certain distance of tag
      if self.__faceDist == True:# and self.__faceState == 3:
	#deltaDist = facedTag.distance - FACEDIST
        #if abs(deltaDist) > 100:
          self.__aim['lx'] = - self.PID_lx.update(facedTag.distance)
        #else:
        #  self.__aim['lx'] = 0

      # update facing state
      #self.__faceStateCounter = self.__faceStateCounter + 1
      #if  abs(self.__aim['az']) < 0.02 and self.__faceState == 0 or \
	  #abs(self.__aim['lz']) < 0.02 and self.__faceState == 1 or \
	  #abs(self.__aim['ly']) < 0.02 and self.__faceState == 2 or \
	  #abs(self.__aim['lx']) < 0.02 and self.__faceState == 3 or \
	  #self.__faceStateCounter > int(CONTROLLRATE*STATEDURATION[0]) and self.__faceState == 0 or \
	  #self.__faceStateCounter > int(CONTROLLRATE*STATEDURATION[1]) and self.__faceState == 1 or \
	  #self.__faceStateCounter > int(CONTROLLRATE*STATEDURATION[2]) and self.__faceState == 2 or \
	  #self.__faceStateCounter > int(CONTROLLRATE*STATEDURATION[3]) and self.__faceState == 3:
	## eiter if one of the states is done, or if too long in one state
	##rospy.loginfo('Face-state = ' + FACESTATES[self.__faceState])
	#self.__faceState = (self.__faceState+1) % 4
	#self.__faceStateCounter = 0
	#self.__onResetSilent(None) # stop moving (silent)

      # sum up correction vectors
      correctionSum = 0.0
      for x in self.__aim.values():
	correctionSum = correctionSum + abs(x)
      rospy.loginfo('correctionSum (%3f)'%correctionSum)
      if correctionSum + self.__lastFaceCorrectionSum < FACESTOPTHRESHOLD:
	status = commandStatus(id=self.__lockNr, isLocked=True, status="face stable")
	self.pub_cmdStatus.publish(status)
	self.__onResetSilent()
      self.__lastFaceCorrectionSum = correctionSum

      # // end designated tag in range

    # keep faceing
    if self.__facedTagNr > -1:
      # call again
      rospy.Timer(rospy.Duration(1.0/CONTROLLRATE), self.__onFace, oneshot=True)


  def Release(self):
    '''
    Action:
      stops faceing of tag (specified by Face(tagNr))
    Parameter:
      tagNr (integer)
    '''
    self.__unlock("released")
    self.__searchState = -1
    if self.__facedTagNr > -1:
      print('released tag nr %i', self.__facedTagNr)
      self.__facedTagNr = -1

  def Search(self, tagNr, startAltd=0):
    '''
    Action:
      Ardrone will yaw slowly, until the specified tag [direction (string)] appears. Overrides all movement controls.
      If it rotated full 360 without finding the target, a warning is prompted.
      [The optional parameter direction may be set "-",changing the direction to "clockwise"]
    Parameter:
      tagNr (integer)
      startAltd (integer) if other than 0, drone will start search at given altitude
    '''
    if self.__lockNr != -1:
      self.lockWarning() # command is not executed, see function description
    else:
      print('Searching ' + str(tagNr))
      self.__searchTagNr = tagNr
      self.__searchState = 0
      self.__searchStepFinished = True
      self.___searchTagNr = tagNr
      if startAltd != 0:
	print('approaching start altitude %i'%startAltd)
	self.__searchStepFinished = False
	self.Altitude(absolute = startAltd, callback = self.__altdStopSearch)
      rospy.Timer(rospy.Duration(1.0/CONTROLLRATE), self.__onSearch, oneshot=True)

  def __onSearch(self, event=None):
    facedTag = self.getTag(self.__searchTagNr)

    if facedTag.diameter != 0:
      # search was successful
      self.__unlock("search successful")
      rospy.loginfo("search successful")
      self.__searchState = 0 # reset search parameter
      self.__searchTagNr = -1
    elif self.__searchState == -1: # or self.__faceLostCounter < 3 :
      # no reason to search
      self.__unlock()
      self.__searchState = 0 # reset search parameter
    elif self.__searchStepFinished == False:
      # wait for step to be done
      rospy.Timer(rospy.Duration(0.2), self.__onSearch, oneshot=True)
    else:
      # keep searching
      if self.__searchStepFinished == True: # only if last step finished
        rospy.loginfo('performing search stage %i: %s'%(self.__searchState, SEARCH[self.__searchState]))
      if self.__reset_twist_timer.isAlive(): # shutdown 'old' timer if still alive (to avoid __onReset by old timer)
        self.__reset_twist_timer.shutdown()

      # perform next move using searchstate values
      yawAngle = SEARCH[self.__searchState][0]
      altdDelta = SEARCH[self.__searchState][1]
      if yawAngle != 0.0:
	# yaw given angle
	self.__aim['az'] = yawAngle/math.fabs(yawAngle) * self.__yawSpeed
	self.__searchStepFinished = False
	durationRecall = rospy.Duration(math.fabs(yawAngle)/(100*self.__yawSpeed) + 1.0) # waiting as long as turn lasts (same code as in Yaw +1.0s)
	durationStop = rospy.Duration(math.fabs(yawAngle)/(100*self.__yawSpeed)) # waiting as long as turn lasts (same code as in Yaw)
	# stop moving
	rospy.Timer(durationStop, self.__onResetSilent, oneshot=True)

	# call again
	rospy.Timer(durationRecall, self.__onSearch, oneshot=True)
      elif altdDelta != 0.0:
	# lift given delta altitude
	self.__searchStepFinished = False
	self.Altitude(delta = altdDelta, callback = self.__altdStopSearch)

      # update search state
      self.__searchState = (self.__searchState + 1)
      if self.__searchState >= SEARCHSTEPS:
	# search was not successful
	rospy.loginfo('search ended')
	self.__unlock("search not successful")
	self.__searchState = -1
	self.__onReset()

  def Approach(self, tagNr):
    '''
    Action:
      Ardrone will automatically search and approach the specified tag as described in 3.2.
      Overrides all movement controls.
    Parameter:
      tagNr (integer)
    '''
    if self.__lockNr != -1:
      self.lockWarning() # command is not executed, see function description
    else:
      print('Approaching ' + str(tagNr))
      self.__approachTagNr = tagNr
      self.__lastApproachTagDistance = 99999999
      self.__approachTagLostCounter = 0
      self.__approachStep = 0
      rospy.Timer(rospy.Duration(1.0/CONTROLLRATE), self.__onApproach, oneshot=True)

  def __onApproach(self, event=None):
    # step 0: gently comming closer ........................
    if self.__approachStep == 0:
      facedTag = self.getTag(self.__approachTagNr)

      if facedTag.diameter == 0:
    	# designated tag is not in range
    	self.__onResetSilent()
    	self.__approachTagLostCounter = self.__approachTagLostCounter + 1
    	rospy.loginfo("tag lost")
      else:
    	# designated tag is in range
    	self.__approachTagLostCounter = 0
    	self.__lastApproachTagDistance = facedTag.distance
    	# find tag centre in relation to screen centre
    	centre = self.getCentre(facedTag)

    	# rotate to face the tag
    	self.__aim['az'] = self.PID_az.update(centre['x'])
    	self.__aim['lx'] = 0.04

      if self.__approachTagLostCounter > 4:
    	# tag is lost for some frames
    	rospy.loginfo("Approach stopped. Eastimated distance is %i"%self.__lastApproachTagDistance)
    	self.__approachStep = 1
      else:
    	# call again
    	rospy.Timer(rospy.Duration(1.0/CONTROLLRATE), self.__onApproach, oneshot=True)

    # step 1: go on top of the box
    if self.__approachStep == 1:
        self.__aim['lx'] = 0.1
        self.__aim['']

  def Navigate(self, x, y, z, xRot, yRot, zRot):
    '''
    Action:
      Performs a graph-based navigation as described in 3.3. Overrides all movement controls.
      Thisfunction it will require an initialization of the graph environment.
    Parameter:
      x, y, z,xRot, yRot, zRot (all integer)
    '''
    if self.__lockNr != -1:
      self.lockWarning() # command is not executed, see function description
    else:
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