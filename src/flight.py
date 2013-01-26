#!/usr/bin/env python

'''----------------------------------------------------------------------------
This is the main file of the project whitin this script the following
services are executed:
 * ardrone driver
 * video output
 * diagnosis
 * input control
-------------------------------------------------------------------------------
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
----------------------------------------------------------------------------'''


# system
import subprocess, time, os, wx, sys, getopt, struct

# wx gui
import wx
import wx.py

# ros
import roslib;
roslib.load_manifest('yudrone')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy, Image
from ardrone_autonomy.msg import Navdata

#local
from commands import *

XLIVEVIEW = 665
YLIVEVIEW = 425
XWINDOW = 1200
YWINDOW = 680
PADDING = 5
  
class FlightFrame(wx.Frame):
  def __init__(self, title):    
    wx.Frame.__init__(self, None, title = title, size=(XWINDOW, YWINDOW))
    self.initGui()
    self.initRos()
  
  '''----------------------------------------------------------------------------
  GUI
  ----------------------------------------------------------------------------'''
  def initGui(self):  
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
    
    
    #create left
    self.imageView = wx.StaticBitmap(self, wx.ID_ANY, wx.BitmapFromImage(wx.EmptyImage(640, 360)), (3*PADDING, 6*PADDING))
    
    self.rbFront = wx.RadioButton(self, 0, 'front-camera', style=wx.RB_GROUP)
    self.rbBottom = wx.RadioButton(self, 0, 'bottom-camera')
    self.rbFront.Bind(wx.EVT_RADIOBUTTON, self.OnRBVideo)
    self.rbBottom.Bind(wx.EVT_RADIOBUTTON, self.OnRBVideo) 
    #self.Bind(wx.EVT_RADIOBUTTON, self.OnRBVideo, id=self.rbFront.GetId()) 
    #self.Bind(wx.EVT_RADIOBUTTON, self.OnRBVideo, id=self.rbBottom.GetId())
    self.rbFront.SetValue(True)
    
    self.rbJoypad = wx.RadioButton(self, 0, 'joypad-only', style=wx.RB_GROUP)
    self.rbConsole = wx.RadioButton(self, 0, 'console')
    self.rbJoypad.Bind(wx.EVT_RADIOBUTTON, self.OnRBInput) 
    self.rbConsole.Bind(wx.EVT_RADIOBUTTON, self.OnRBInput)
    #self.Bind(wx.EVT_RADIOBUTTON, self.OnRBInput, id=self.rbJoypad.GetId()) 
    #self.Bind(wx.EVT_RADIOBUTTON, self.OnRBInput, id=self.rbConsole.GetId())
    self.rbJoypad.SetValue(True)
    
    self.CommandList = ['Altitude(', 'MaxAlt(', 'MinAlt(', 'Yaw(', 'YawSpeed(',
		 'Horizontal(', 'HrzSpeed(', 'TakeOff()', 'Land()',
		 'Pause()', 'Continue()', 'Break()', 'Batch(', 'Exit()',
		 'ToggleEmerg()', 'Face(', 'Release()', 'Search(',
		 'Approach(', 'Navigate(']    
    self.cmbCommands = wx.ComboBox(self, 0, choices=self.CommandList, style=wx.CB_READONLY)
    self.Bind(wx.EVT_COMBOBOX, self.OnSelectCmd, id=self.cmbCommands.GetId())
    self.cmdShell = wx.py.shell.Shell(self)
    
    #assemble left
    sizeRadioBtn1.Add(self.rbFront, 0, wx.ALL, 0)
    sizeRadioBtn1.Add(self.rbBottom, 0, wx.ALL, 0)
    sizeRadioBtn2.Add(self.rbJoypad, 0, wx.ALL, 0)
    sizeRadioBtn2.Add(self.rbConsole, 0, wx.ALL, 0)
    
    sizeLeft.Add(wx.StaticText(self), 0, wx.ALL, 0) #spacer
    sizeLeft.Add(self.imageView, 0 ,wx.ALL, 10)
    sizeLeft.Add(sizeRadioBtn1, 0 ,wx.CENTER, 5)
    
    sizeLeft.Add(wx.StaticText(self), 0, wx.ALL, 5) #spacer
    sizeLeft.Add(self.cmbCommands, 0, wx.EXPAND, 10)
    sizeLeft.Add(self.cmdShell,0,wx.EXPAND,5)
    sizeLeft.Add(sizeRadioBtn2, 0 ,wx.CENTER, 10)
    
    
    #create right
    self.txtConnection = wx.StaticText(self) #spacer
    self.txtBattery = wx.StaticText(self)
    self.txtState = wx.StaticText(self)
    self.txtAltd = wx.StaticText(self)
    
    self.txtConnection.SetLabel('Connection: NONE')
    self.txtBattery.SetLabel('Battery: UNKNOWN')
    self.txtState.SetLabel('Status: UNKNOWN')
    self.txtAltd.SetLabel('Altitude (mm): ')
    
    #assemble right
    sizeRight.Add(wx.StaticText(self), 0, wx.ALL, 5)
    sizeRight.Add(self.txtConnection, 0, wx.ALL, 5)
    sizeRight.Add(self.txtBattery, 0, wx.ALL, 5)
    sizeRight.Add(self.txtState, 0, wx.ALL, 5)
    sizeRight.Add(self.txtAltd, 0, wx.ALL, 5)
    
    #finalize
    sizeAll.Add(sizeLeft, 0, wx.ALL, 5)
    sizeAll.Add(sizeRight, 0, wx.ALL, 5)
    self.SetSizer(sizeAll)    
    self.Bind(wx.EVT_CLOSE, self.OnClose)
    print('GUI initialized')

  def OnClose(self, event='NONE'):
    dlg = wx.MessageDialog(self, "Do you really want to close this application?", "Confirm Exit", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
    result = dlg.ShowModal()
    dlg.Destroy()
    if result == wx.ID_YES:
      self.Destroy()
      print 'Shutting down'
      
    
  '''----------------------------------------------------------------------------
  ROS
  ----------------------------------------------------------------------------'''
  def initRos(self):
    roscoreRunning=False
    try:
      #check master
      master = rospy.get_master() 
      master.getSystemState()
      roscoreRunning=True
    except:
      dlg = wx.MessageDialog(self,
	  "Yudrone cannot work without roscore. Should it be executed in yudrone?",
	  "Roscore is not running!", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
      result = dlg.ShowModal()
      dlg.Destroy()
      if result == wx.ID_YES:
	self.openRoscore()
	try:
	  #check master
	  master = rospy.get_master() 
	  master.getSystemState()
	  roscoreRunning=True
	except:
	  roscoreRunning=False
	
    if roscoreRunning == True:
      #create own node    
      rospy.init_node('yudrone_flight')
      
      #publishers
      self.pub_log = rospy.Publisher( "yudrone/log", String )
      self.pub_land = rospy.Publisher( "ardrone/land", Empty )
      self.pub_takeoff = rospy.Publisher( "ardrone/takeoff", Empty )
      self.pub_reset = rospy.Publisher( "ardrone/reset", Empty )
      self.pub_yaw = rospy.Publisher( "/cmd_vel", Twist )
      
      #subscribe
      self.sub_joy = rospy.Subscriber( "joy", Joy, self.handle_joy )
      self.sub_nav = rospy.Subscriber( "ardrone/navdata", Navdata, self.handle_navdata )
      self.sub_log = rospy.Subscriber( "yudrone/log", String, self.handle_log )
      self.sub_image = rospy.Subscriber("/ar/image", Image,self.handle_image)
      time.sleep(0.1)
      
      #variables
      self.state='front'
      
      #finalize
      self.OnRBInput()
      self.OnRBVideo()
      self.pub_log.publish('ROS initialized')
      
    else:
      print('No roscore; shutting down')
      self.Destroy()
       
    
  def handle_log(self, log):
    print(log.data)
    
  def openDriver(self):
    self.pub_log.publish("+++ OPENING DRIVER ++++++++++++++++++++++++++++")
    self.pipe_driver=subprocess.Popen(["rosrun","ardrone_autonomy","ardrone_driver"])
    time.sleep(10)
  
  def openRoscore(self):
    print('+++ OPENING ROSCORE +++++++++++++++++++++++++++')
    self.pipe_driver=subprocess.Popen(["roscore"])
    time.sleep(5)
    
  '''----------------------------------------------------------------------------
  VIDEO
  ----------------------------------------------------------------------------'''     
  def handle_image(self, data):
    wx.CallAfter(self.updateImage, data)
    
  def updateImage(self, data):
    #print('updateimage')
    img = wx.ImageFromData(data.width, data.height, data.data)
    bitmap = wx.BitmapFromImage(img)
    self.imageView.SetBitmap(bitmap)
    
  def OnRBVideo(self, event='NONE'):
    if self.rbFront.GetValue() == True and self.state == 'bottom':
      subprocess.call(["rosservice","call","/ardrone/togglecam"])
      self.state = 'front'
      self.pub_log.publish('Frontcam activated')
    if self.rbBottom.GetValue() == True and self.state == 'front':
      subprocess.call(["rosservice","call","/ardrone/togglecam"])
      self.state = 'bottom'
      self.pub_log.publish('Bottomcam activated') 
    
  '''----------------------------------------------------------------------------
  DIAGNISTICS
  ----------------------------------------------------------------------------'''            
  def handle_navdata(self, navdata):
    self.txtBattery.SetLabel("Battery: " + str(navdata.batteryPercent))
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
      currStatus='unknown'
    self.txtState.SetLabel('Status: ' + currStatus)
    self.txtAltd.SetLabel('Altitude (mm): ' + str(navdata.altd))
	
  def handle_log(self, log):
    print(log.data)
    
  '''----------------------------------------------------------------------------
  INPUT
  ----------------------------------------------------------------------------'''
  def OnRBInput(self, event='NONE'):
    if self.rbJoypad.GetValue() == True:
      self.pub_log.publish('Input set to joypad')
      self.rbFront.Enable()
      self.rbBottom.Enable()
      self.cmdShell.Disable()
      self.cmbCommands.Disable()
    else:
      self.pub_log.publish('Input set to console')
      self.rbFront.Disable()
      self.rbBottom.Disable()
      self.cmdShell.Enable()
      self.cmbCommands.Enable()
      
  def handle_joy(self, joy):
    if self.btnState.get() == 'joypad':
      # btn nr  for takeoff
      if joy.buttons[3]==1:# and status["state"] == 'ground':
	self.pub_log.publish('start command')
	self.pub_takeoff.publish( Empty() )
	time.sleep(0.1)
      # btn nr  for land
      if joy.buttons[1]==1:# and status["state"] == 'flight':
	self.pub_log.publish('land command')
	self.pub_land.publish( Empty() )
	time.sleep(0.1)
      # btn nr  for reset
      if joy.buttons[0]==1:
	self.pub_log.publish('reset command')
	self.pub_reset.publish( Empty() )
	time.sleep(0.1)
    
      # set yaw parameter
      yaw = Twist()
      yaw.angular.x = yaw.angular.y = 0
      yaw.angular.z = joy.axes[2] * 3.14/2
      yaw.linear.z = joy.axes[3] * 2.0
      yaw.linear.x = joy.axes[1] * 1.0
      yaw.linear.y = joy.axes[0] * 1.0
      
      # publish yaw to ardrone
      self.pub_yaw.publish(yaw)
      
  def OnSelectCmd(self, event):
    item = event.GetSelection()
    self.cmdShell.setFocus()
    self.cmdShell.clearCommand()
    self.cmdShell.write(self.CommandList[item])
    self.cmdShell.autoCallTipShow(self.CommandList[item])
    
  
'''----------------------------------------------------------------------------
top-level-code
STARTS application
----------------------------------------------------------------------------'''
if __name__ == '__main__':
  # start application
  app = wx.App(redirect=False)
  top = FlightFrame('yudrone flight')
  top.Show(True)
  app.MainLoop()