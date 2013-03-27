#!/usr/bin/env python

'''************************************************************************************************************************
This is the main file of the project whitin this script the following
services are executed:
 * ardrone driver
 * video output
 * joypad_ctrl
 * diagnosis
 * command based control
***************************************************************************************************************************
TODO
* add watchdog **> https://mediabox.grasp.upenn.edu/svn/penn-ros-pkgs/pr2_props_kinect_stack/trunk/props_openni_tracker/scripts/openni_watchdog.py
* note camera may stop --> watchdog on all incomming topics
***************************************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
************************************************************************************************************************'''

# system
import subprocess, os, sys, getopt, struct
import threading
import thread

# wx gui
import wx
import wx.py

# ros
import roslib;
roslib.load_manifest('yudrone')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String, Bool
from sensor_msgs.msg import Joy, Image
from ardrone_autonomy.msg import Navdata
from ar_recog.msg import Tags, Tag

#local
import commands
from singleton import SingletonType
import plot
from setDefaultParameter import *

XLIVEVIEW = 665
YLIVEVIEW = 425
XWINDOW = 1200
YWINDOW = 720
PADDING = 5

'''************************************************************************************************************************
main class of project (controller & view)
************************************************************************************************************************''' 
class Flight(wx.Frame):
  __metaclass__ = SingletonType
  
  def __init__(self, title='yudrone flight', plotRot=False, plotMag=False):
    '''
    Constructor
    '''
    wx.Frame.__init__(self, None, wx.NewId(), title = title, size=(XWINDOW, YWINDOW))
    self.__plotRot = plotRot
    self.__plotMag = plotMag
    self.droneState = 2 # landed
    self.shell = None # initialize command class as NONE
    self.__initGui()
    self.__initRos()
      
  '''************************************************************************************************************************
  GUI
  ************************************************************************************************************************'''
  def __initGui(self):  
    '''
    inits the yudrone GUI
    * layout
    * left + right widgets
    '''
  
    # stage 1: layout ................................................
    #boxes and borders
    wx.StaticBox(self, -1, 'LiveView', (PADDING, PADDING), size=(XLIVEVIEW, YLIVEVIEW))
    wx.StaticBox(self, -1, 'Diagnostics', (XLIVEVIEW + PADDING, PADDING), size=(XWINDOW - XLIVEVIEW - 2*PADDING, YWINDOW-2*PADDING))
    wx.StaticBox(self, -1, 'Input', (PADDING, YLIVEVIEW + PADDING), size=(XLIVEVIEW, YWINDOW-YLIVEVIEW-2*PADDING))
    
    #create sizer
    sizeAll = wx.BoxSizer(wx.HORIZONTAL)
    sizeLeft = wx.BoxSizer(wx.VERTICAL)
    sizeRight = wx.BoxSizer(wx.VERTICAL)
    sizeRadioBtn1 = wx.BoxSizer(wx.HORIZONTAL)
    sizeRadioBtn2 = wx.BoxSizer(wx.HORIZONTAL)
    sizeCli = wx.BoxSizer(wx.HORIZONTAL)
    
    # stage 2: left widgets ........................................
    #create left
    self.imageView = wx.StaticBitmap(self, wx.ID_ANY, wx.BitmapFromImage(wx.EmptyImage(640, 360)), (3*PADDING, 6*PADDING))
    
    self.rbFront = wx.RadioButton(self, 0, 'front-camera', style=wx.RB_GROUP)
    self.rbBottom = wx.RadioButton(self, 0, 'bottom-camera')
    self.rbOff = wx.RadioButton(self, 0, 'off')
    self.rbFront.Bind(wx.EVT_RADIOBUTTON, self.OnRBVideo)
    self.rbBottom.Bind(wx.EVT_RADIOBUTTON, self.OnRBVideo) 
    self.rbOff.Bind(wx.EVT_RADIOBUTTON, self.OnRBVideo)
    self.rbOff.SetValue(True)
    
    self.rbJoypad = wx.RadioButton(self, 0, 'joypad-only', style=wx.RB_GROUP)
    self.rbConsole = wx.RadioButton(self, 0, 'console')
    self.rbJoypad.Bind(wx.EVT_RADIOBUTTON, self.OnRBInput) 
    self.rbConsole.Bind(wx.EVT_RADIOBUTTON, self.OnRBInput)
    self.rbJoypad.SetValue(True)
    
    self.CommandList = ['Altitude(', 'MaxAltd(', 'MinAltd(', 'Yaw(', 'YawSpeed(',
		 'Horizontal(', 'HrzSpeed(', 'TakeOff()', 'Land()',
		 'Pause()', 'Continue()', 'Break()', 'Batch(', 'Exit()',
		 'ToggleEmerg()', 'Face(', 'Release()', 'Search(',
		 'Approach(', 'Navigate(']    
    self.cmbCommands = wx.ComboBox(self, 0, choices=self.CommandList, style=wx.CB_READONLY)
    self.Bind(wx.EVT_COMBOBOX, self.OnSelectCmd, id=self.cmbCommands.GetId())
    self.cmdShell = wx.py.shell.Shell(self, size =(XLIVEVIEW, YLIVEVIEW / 2))
    
    #assemble left
    sizeRadioBtn1.Add(self.rbFront, 0, wx.ALL, 0)
    sizeRadioBtn1.Add(self.rbBottom, 0, wx.ALL, 0)
    sizeRadioBtn1.Add(self.rbOff, 0, wx.ALL, 0)
    sizeRadioBtn2.Add(self.rbJoypad, 0, wx.ALL, 0)
    sizeRadioBtn2.Add(self.rbConsole, 0, wx.ALL, 0)
    
    sizeLeft.Add(wx.StaticText(self), 0, wx.ALL, 0) #spacer
    sizeLeft.Add(self.imageView, 0 ,wx.ALL, 10)
    sizeLeft.Add(sizeRadioBtn1, 0 ,wx.CENTER, 5)
    
    sizeLeft.Add(wx.StaticText(self), 0, wx.ALL, 5) #spacer
    sizeLeft.Add(self.cmbCommands, 0, wx.EXPAND, 10)
    sizeLeft.Add(self.cmdShell,0,wx.EXPAND,5)
    sizeLeft.Add(sizeRadioBtn2, 0 ,wx.CENTER, 10)
    
    # stage 3: right widgets .......................................
    #create right
    self.txtConnection = wx.StaticText(self) #spacer
    self.txtBattery = wx.StaticText(self)
    self.txtState = wx.StaticText(self)
    self.txtAltd = wx.StaticText(self)
    
    self.txtRotation = wx.StaticText(self)
    self.txtMagnetometer = wx.StaticText(self)
    self.txtAim = wx.StaticText(self)
    self.txtTags = wx.StaticText(self)
    if self.__plotRot == True:
      self.txtPlotA= wx.StaticText(self) # graph text
      self.plotA = plot.GraphPanel(self) # graph
      self.txtPlotA.SetLabel('Rotation-Graph XYZ (rgb) ')
    if self.__plotMag == True:
      self.txtPlotB= wx.StaticText(self) # graph text
      self.plotB = plot.GraphPanel(self) # graph
      self.txtPlotB.SetLabel('Magnetometer-Graph XYZ (rgb) ')
    
    self.txtConnection.SetLabel('Connection:\t\tNONE')
    self.txtBattery.SetLabel('Battery:\t\t\tUNKNOWN')
    self.txtState.SetLabel('Status:\t\t\tUNKNOWN')
    self.txtAltd.SetLabel('Altitude (mm):')
    
    self.txtRotation.SetLabel('Rotation: ')
    self.txtMagnetometer.SetLabel('Magnetometer: ')
    self.txtAim.SetLabel('Aim: ')
    self.txtTags.SetLabel('Tags: ')
    
    #assemble right
    sizeRight.Add(wx.StaticText(self), 0, wx.ALL, 5)
    sizeRight.Add(self.txtConnection, 0, wx.ALL, 5)
    sizeRight.Add(self.txtBattery, 0, wx.ALL, 5)
    sizeRight.Add(self.txtState, 0, wx.ALL, 5)
    sizeRight.Add(self.txtAltd, 0, wx.ALL, 5)
    sizeRight.Add(self.txtRotation, 0, wx.ALL, 5)
    sizeRight.Add(self.txtMagnetometer, 0, wx.ALL, 5)
    sizeRight.Add(self.txtAim, 0, wx.ALL, 5)
    sizeRight.Add(self.txtTags, 0, wx.ALL, 5)
    if self.__plotRot == True:
      sizeRight.Add(self.txtPlotA, 0, wx.EXPAND, 5)
      sizeRight.Add(self.plotA, 0, wx.EXPAND, 5)
    if self.__plotMag == True:
      sizeRight.Add(self.txtPlotB, 0, wx.EXPAND, 5)
      sizeRight.Add(self.plotB, 0, wx.EXPAND, 5)
    
    # stage 4: finalize ............................................
    #finalize
    sizeAll.Add(sizeLeft, 0, wx.ALL, 5)
    sizeAll.Add(sizeRight, 0, wx.ALL, 5)
    self.SetSizer(sizeAll)    
    self.Bind(wx.EVT_CLOSE, self.OnClose)
    rospy.loginfo('GUI initialized')
    rospy.loginfo('yudrone is ready...')

  def __del__(self):
    self.OnClose()
    
  def OnClose(self, event='NONE'):
    '''
    Callback function for closing application
    '''
    if self.droneState != 2 or self.droneState != 0:
      self.pub_emergency.publish( Empty() )
    #dlg = wx.MessageDialog(self, "Do you really want to close this application?", "Confirm Exit", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
    #result = dlg.ShowModal()
    #dlg.Destroy()
    #if result == wx.ID_YES:
    self.Destroy()
    if self.camState == 'bottom':
      subprocess.call(["rosservice","call","/ardrone/togglecam"])
    rospy.signal_shutdown('closing yudrone')
    rospy.loginfo('Shutting down')
      
    
  '''************************************************************************************************************************
  ROS
  ************************************************************************************************************************'''
  def __initRos(self):
    '''
    Initializes Ros environment including
    * roscore
    * ardrone_driver
    * joy_node
    * ar_toolkit
    * own node
    '''
    
    # .............................................................................
    rospy.loginfo('stage 1: check roscore ....') 
    # .............................................................................
    roscoreRunning=False
    try:
      #check master
      master = rospy.get_master() 
      master.getSystemState()
      roscoreRunning=True
    except:
      dlg = wx.MessageDialog(self,
        "Roscore is not running. Yudrone cannot work without roscore. Should it be executed by yudrone?",
        "Roscore is not running!", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
      result = dlg.ShowModal()
      dlg.Destroy()
      if result == wx.ID_YES:
	# open roscore as subprocess
	self.openRoscore()
    try:
      #check master again
      master = rospy.get_master() 
      master.getSystemState()
      roscoreRunning=True
    except:
      roscoreRunning=False
    
    if roscoreRunning == False:
      rospy.logfatal('No sroscore; shutting down')
      self.Destroy()
    rospy.loginfo('Roscore is running')
    # load parameters
    setDefaultParameter()
    
    # .............................................................................
    rospy.loginfo('stage 2: check network ....') 
    # .............................................................................
    ARDRONE_IP = rospy.get_param('yudrone/ARDRONE_IP')
    self.__ROSDIR = rospy.get_param('yudrone/ROSDIR')
    networkConnected = os.system('ping ' + ARDRONE_IP + ' -c 4 -i 0.2 -w 3 > /dev/null')
    if networkConnected != 0:
      rospy.logerr('not connected to ardrone (IP: ' + ARDRONE_IP + ')')
      dlg = wx.MessageDialog(self,
        "There is no ardrone connected (at IP= " + ARDRONE_IP + ")",
        "Network error", wx.OK|wx.ICON_ERROR)
      result = dlg.ShowModal()
      dlg.Destroy()
    else:
      rospy.loginfo('connected to ardrone (IP: ' + ARDRONE_IP + ')')
    
    # .............................................................................
    rospy.loginfo('stage 3: check ardrone_driver ...') 
    # .............................................................................
    # check node
    self.ardroneDriverRunnig = os.system('rosnode list | grep /ardrone > /dev/null')
    if self.ardroneDriverRunnig != 0:
      # no driver
      dlg = wx.MessageDialog(self,
        "Ardrone_driver is not running. Should it be executed by yudrone?",
        "Ardrone_driver is not running!", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
      result = dlg.ShowModal()
      dlg.Destroy()
      if result == wx.ID_YES:
        # open roscore as subprocess
        self.openDriver()
    
    # check node again
    self.ardroneDriverRunnig = os.system('rosnode list | grep /ardrone > /dev/null')
    if self.ardroneDriverRunnig != 0:
      rospy.logerr('ardrone_driver is not running')
    else:
      rospy.loginfo('ardrone_driver is running')
    self.navdataCounter=0
    
    # .............................................................................  
    rospy.loginfo('stage 4: check joy ....') 
    # .............................................................................
    # check joy_node
    joyNodeRunning = os.system('rosnode list | grep /joy_node > /dev/null')
    if joyNodeRunning != 0:
      # no joy_node
      dlg = wx.MessageDialog(self,
        "Joynode is not running. Should it be executed by yudrone?",
        "Joynode is not running!", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
      result = dlg.ShowModal()
      dlg.Destroy()
      if result == wx.ID_YES:
        # open joy_node as subprocess
        self.openJoyNode()
    
    # check joynode again
    joyNodeRunning = os.system('rosnode list | grep /joy_node > /dev/null')
    if joyNodeRunning != 0:
      rospy.logerr('joynode is not running')
    else:
      rospy.loginfo('joynode is running')
    
    # check yudrone_joy
    joypadCtrlRunning = os.system('rosnode list | grep /yudrone_joy > /dev/null')
    if joypadCtrlRunning != 0:
      # no joy_node
      dlg = wx.MessageDialog(self,
        "yudrone_joy is not running. Should it be executed by yudrone?",
        "yudrone_joy is not running!", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
      result = dlg.ShowModal()
      dlg.Destroy()
      if result == wx.ID_YES:
        # open joy_node as subprocess
        self.openJoyCtrl()
    
    # check yudrone_joy again
    joypadCtrlRunning = os.system('rosnode list | grep /yudrone_joy > /dev/null')
    if joypadCtrlRunning != 0:
      rospy.logerr('yudrone_joy is not running')
    else:
      rospy.loginfo('yudrone_joy is running')
    
    # .............................................................................
    rospy.loginfo('stage 5: check ar_toolkit ....') 
    # .............................................................................
    arNodeRunning = os.system('rostopic list | grep /ar/image > /dev/null')
    if arNodeRunning != 0:
      # no ar_node
      dlg = wx.MessageDialog(self,
        "Ar_toolkit is not running. Should it be executed by yudrone?",
        "Ar_toolkit is not running!", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
      result = dlg.ShowModal()
      dlg.Destroy()
      if result == wx.ID_YES:
	# open ar-node as subprocess
	self.openArToolkit()
    
    # check ar_node again
    arNodeRunning = os.system('rostopic list | grep /ar/image > /dev/null')
    if arNodeRunning != 0:
      rospy.logerr('ar_node is not running')
      self.__imageTopic = '/ardrone/image_raw'
    else:
      rospy.loginfo('ar_node is running')
      self.__imageTopic = '/ar/image'
    
    # .............................................................................
    rospy.loginfo('stage 6: init own node ...') 
    # .............................................................................
    #create node
    rospy.init_node('yudrone_flight')
    
    #publishers
    self.pub_emergency = rospy.Publisher( "ardrone/reset", Empty )		# toggle ardrone emergency state
    self.pub_lockJoy = rospy.Publisher( "/yudrone/lock_joypad", Bool )	# lock axis of joypad
    
    #subscribers
    self.sub_nav = rospy.Subscriber( "ardrone/navdata", Navdata, self.handle_navdata )
    self.sub_tags = rospy.Subscriber( "tags", Tags, self.handle_tags )	# obtain tags (only for screen output)
    self.sub_yaw = rospy.Subscriber( "/cmd_vel", Twist, self.handle_twist )	# obtain twist msgs (only for screen output)
    self.sub_image = None
    rospy.sleep(0.1)
    
    #variables
    self.camState='front' # initially the camera is set to front on ardrone
    
    #stage 7: finalize ............................................
    self.OnRBInput()
    self.OnRBVideo()
    self.watchdog = rospy.Timer(rospy.Duration(1), self.__watchdog)
    self.currYaw = Twist()
    rospy.loginfo('ROS initialized')
    
  def openDriver(self):
    '''
    opens ardrone_driver in a subprocess
    '''
    rospy.loginfo("open driver...")
    cmd= self.__ROSDIR + "rosrun ardrone_autonomy ardrone_driver"
    self.pipe_ros=subprocess.Popen(['gnome-terminal', '--command='+cmd])
    rospy.sleep(10)
  
  def openRoscore(self):
    '''
    opens ardrone_driver in a subprocess
    '''
    rospy.loginfo('open roscore...')
    cmd="/opt/ros/fuerte/bin/roscore"
    self.pipe_ros=subprocess.Popen(['gnome-terminal', '--command='+cmd])
    rospy.sleep(3)
    
  def openJoyNode(self):
    '''
    opens joy_node in a subprocess
    '''
    rospy.loginfo('open joynode...')
    cmd=self.__ROSDIR + "rosrun joy joy_node"
    self.pipe_joy=subprocess.Popen(['gnome-terminal', '--command='+cmd])
    rospy.sleep(3)
    
  def openJoyCtrl(self):
    '''
    opens yudrone_joy in a subprocess
    '''
    rospy.loginfo('open yudrone_joy...')
    cmd=self.__ROSDIR + "rosrun yudrone joypad_ctrl.py"
    self.pipe_joy=subprocess.Popen(['gnome-terminal', '--command='+cmd])
    rospy.sleep(3)
    
  def openArToolkit(self):
    '''
    opens ar_recog in a subprocess
    '''
    rospy.loginfo('open ar_toolkit...')
    ARRECOGDIR = rospy.get_param('yudrone/ARRECOGDIR')
    rospy.set_param('aov', 0.67)
    cmd=self.__ROSDIR + "rosrun ar_recog ar_recog image:=/ardrone/image_raw"
    self.pipe_ar=subprocess.Popen(['gnome-terminal', '--command='+cmd, '--working-directory=' + ARRECOGDIR])
    rospy.sleep(1)
    
  '''************************************************************************************************************************
  VIDEO
  ************************************************************************************************************************'''     
  def handle_image(self, data):
    '''
    Callback function for incomming video frames
    '''
    wx.CallAfter(self.updateImage, data) # call asynchon
    
  def updateImage(self, data):
    '''
    Callback function for incomming video frames
    '''
    img = wx.ImageFromData(data.width, data.height, data.data)
    bitmap = wx.BitmapFromImage(img)
    self.imageView.SetBitmap(bitmap)
    
  def OnRBVideo(self, event='NONE'):
    '''
    Callback function for changing video radiobuttons
    '''
    # unregister
    if self.sub_image != None:
      self.sub_image.unregister()
      self.sub_image = None
    
    # set values for resolution and camState
    if self.rbFront.GetValue() == True and self.camState == 'bottom':
      subprocess.call(["rosservice","call","/ardrone/togglecam"])
      self.camState = 'front'
      self.shell.updateResolution(360, 640) 
      rospy.loginfo('Frontcam activated')
    if self.rbBottom.GetValue() == True and self.camState == 'front':
      subprocess.call(["rosservice","call","/ardrone/togglecam"])
      self.camState = 'bottom'
      self.shell.updateResolution(360, 640)
      rospy.loginfo('Bottomcam activated')
      
    # on or off
    if self.rbOff.GetValue() == True:
      rospy.loginfo('Camera deactivated')
    else:
      self.sub_image = rospy.Subscriber(self.__imageTopic, Image,self.handle_image)
    
    
  '''************************************************************************************************************************
  DIAGNISTICS
  ************************************************************************************************************************'''  
  def setCommands(self, shell):
    self.shell = shell 
    self.shell.updateResolution(360, 640) 
   
  def __watchdog(self, event = None):
    self.ardroneDriverRunnig = os.system('rosnode ping -c2 /ardrone_driver | grep "xmlrpc reply from" > /dev/null')
  
  def printNavdata(self, navdata):
    '''
    Callback function for navigation data comming from ardrone
    '''
    if self.navdataCounter % 10 == 0:
      if self.ardroneDriverRunnig == 0:
        self.txtConnection.SetLabel('Connection:\t\tGOOD')
      else:
        self.txtConnection.SetLabel('Connection:\t\tLOST')
    
      self.txtBattery.SetLabel("Battery:\t\t\t%d" %navdata.batteryPercent)
      
      if navdata.state == 1:
        currStatus='inited'
      elif navdata.state == 2:
        currStatus='landed'
      elif navdata.state == 3 or navdata.state == 7:
        currStatus='flying'
      elif navdata.state == 4:
        currStatus='hovering'
      elif navdata.state == 6:
        currStatus='taking off'
      elif navdata.state == 8:
        currStatus='landing'
      else:
        currStatus='emergency'
      self.txtState.SetLabel('Status:\t\t\t' + currStatus)    
      
      self.txtAltd.SetLabel('Altitude (mm):\t%d' %navdata.altd)
      self.txtRotation.SetLabel('Rotation:\t\t\tX:%d\tY:%d\tZ:%d' %(navdata.rotX, navdata.rotY, navdata.rotZ))
      self.txtMagnetometer.SetLabel('Magnetometer:\tX:%d\tY:%d\tZ:%d' %(navdata.magX, navdata.magY, navdata.magZ))
      self.txtAim.SetLabel('Aim:\t\t\t\tLX:%.2f\tLY:%.2f\tLZ:%.2f\tAZ:%.2f' %(self.currYaw.linear.x, self.currYaw.linear.y, self.currYaw.linear.z, self.currYaw.angular.z))
      self.droneState = navdata.state
      
      if self.__plotRot == True:
        wx.CallAfter(self.plotA.handle_xyz, {'x':navdata.rotX, 'y':navdata.rotY, 'z':navdata.rotZ})
        self.plotA.handle_xyz({'x':navdata.rotX, 'y':navdata.rotY, 'z':navdata.rotZ})
      if self.__plotMag == True:
        wx.CallAfter(self.plotB.handle_xyz, {'x':navdata.magX, 'y':navdata.magY, 'z':navdata.magZ})
        self.plotB.handle_xyz({'x':navdata.magX, 'y':navdata.magY, 'z':navdata.magZ})
  
  def handle_twist(self, yaw):
    # just for data output
    self.currYaw = yaw
  
  def handle_navdata(self, navdata):
    self.navdataCounter += 1
    wx.CallAfter(self.printNavdata, navdata)

  def print_tags(self, msg):
    if self.navdataCounter%10 == 0:
      nrOfTags = msg.tag_count
      string = ""
      for tag in msg.tags:
        string += "[%d] X:%d  Y:%d  D:%d\t" %(tag.id, tag.x, tag.y, tag.distance)
      self.txtTags.SetLabel('Tags:\t\t\t' + string)
      
  def handle_tags(self, msg):
    wx.CallAfter(self.print_tags, msg)
    
  '''************************************************************************************************************************
  INPUT
  ************************************************************************************************************************'''
  def OnRBInput(self, event='NONE'):
    '''
    Callback function for changing input radiobuttons
    '''
    if self.rbJoypad.GetValue() == True:
      rospy.loginfo('Input set to joypad')
      self.rbFront.Enable()
      self.rbBottom.Enable()
      self.rbOff.Enable()
      self.cmdShell.Disable()
      self.cmbCommands.Disable()
      self.pub_lockJoy.publish(False)
      if not self.shell is None:
        self.shell.updateNavSwitch(False)
    else:
      rospy.loginfo('Input set to console')
      self.rbFront.Disable()
      self.rbBottom.Disable()
      self.rbOff.Disable()
      self.cmdShell.Enable()
      self.cmbCommands.Enable()
      self.pub_lockJoy.publish(True)
      if not self.shell is None:
        self.shell.updateNavSwitch(True)
    
  def OnSelectCmd(self, event):    
    '''
    Callback function for selecting command from combobox
    '''
    item = event.GetSelection()
    self.cmdShell.setFocus()
    self.cmdShell.clearCommand()
    self.cmdShell.write("shell." + self.CommandList[item])
    self.cmdShell.autoCallTipShow("shell." + self.CommandList[item])
    
def reloadCommands():
  #reloads command file
  reload(commands)
  shell = commands.Commands(flight)
  flight.setCommands(shell)
  
'''************************************************************************************************************************
top-level-code
STARTS application
************************************************************************************************************************'''
if __name__ == '__main__':
  try:
    # start application
    app = wx.App(redirect=False)
    flight = Flight()
    shell = commands.Commands(flight)
    flight.setCommands(shell)
    flight.Show(True)
    app.MainLoop()
    #if droneState != 2 or droneState != 0:
    #  param=" pub /ardrone/reset std_msgs/Empty --once"
    #  subprocess.Popen(['/opt/ros/fuerte/bin/rostopic', param])
  except:
    #if droneState != 2 or droneState != 0:
    #  param=" pub /ardrone/reset std_msgs/Empty --once"
    #  subprocess.Popen(['/opt/ros/fuerte/bin/rostopic', param])
    # todo stop drone
    print 'Exception occured'
    for line in sys.exc_info():
      print(line)

    