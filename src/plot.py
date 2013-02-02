#!/usr/bin/env python

'''************************************************************************************************************************

***************************************************************************************************************************
TODO

***************************************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
************************************************************************************************************************'''

# wx gui
import wx
import wx.py

# ros
import roslib.messages
import roslib.names
import rospy
import rosgraph
from ardrone_autonomy.msg import Navdata


class Flight(wx.Frame):
  __metaclass__ = SingletonType
  
  def __init__(self, title='yudrone flight'):
    '''
    Constructor
    '''
    wx.Frame.__init__(self, None, title = title, size=(XWINDOW, YWINDOW))
    self.droneState = 2 # landed
    self.shell = None
    self.initGui()
    self.initRos()
    
      
  '''************************************************************************************************************************
  GUI
  ************************************************************************************************************************'''
  def initGui(self):  
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
    self.cmdShell = wx.py.shell.Shell(self, size =(XLIVEVIEW, YLIVEVIEW / 2))
    
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
    
    # stage 3: right widgets .......................................
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
    
    # stage 4: finalize ............................................
    #finalize
    sizeAll.Add(sizeLeft, 0, wx.ALL, 5)
    sizeAll.Add(sizeRight, 0, wx.ALL, 5)
    self.SetSizer(sizeAll)    
    self.Bind(wx.EVT_CLOSE, self.OnClose)
    rospy.loginfo('GUI initialized')
    rospy.loginfo('yudrone is ready...')

  def OnClose(self, event='NONE'):
    '''
    Callback function for closing application
    '''
    if self.droneState != 2 or self.droneState != 0:
      self.pub_emergency.publish( Empty() )
    dlg = wx.MessageDialog(self, "Do you really want to close this application?", "Confirm Exit", wx.YES_NO|wx.YES_DEFAULT|wx.ICON_QUESTION)
    result = dlg.ShowModal()
    dlg.Destroy()
    if result == wx.ID_YES:
      self.Destroy()
      rospy.signal_shutdown('closing yudrone')
      rospy.loginfo('Shutting down')
      
    
  '''************************************************************************************************************************
  ROS
  ************************************************************************************************************************'''
  def initRos(self):
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
      rospy.logfatal('No roscore; shutting down')
      self.Destroy()
    rospy.loginfo('Roscore is running')
    
    # .............................................................................
    rospy.loginfo('stage 2: check network ....') 
    # .............................................................................
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
    ardroneDriverRunnig = os.system('rosnode list | grep /ardrone > /dev/null')
    if ardroneDriverRunnig != 0:
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
    ardroneDriverRunnig = os.system('rosnode list | grep /ardrone > /dev/null')
    if ardroneDriverRunnig != 0:
      rospy.logerr('ardrone_driver is not running')
    else:
      rospy.loginfo('ardrone_driver is running')
    
    # .............................................................................  
    rospy.loginfo('stage 4: check joy ....') 
    # .............................................................................
    joypadConnected = os.system('test -e /dev/input/js2')
    if joypadConnected != 0:
      rospy.logerr('no joypad connected')
    else:
      rospy.loginfo('joypad is connected')
    
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
      imageTopic = '/ardrone/image_raw'
    else:
      rospy.loginfo('ar_node is running')
      imageTopic = '/ar/image'
    
    # .............................................................................
    rospy.loginfo('stage 6: init own node ...') 
    # .............................................................................
    #create node
    rospy.init_node('yudrone_flight')
    
    #publishers
    self.pub_log = rospy.Publisher( "yudrone/log", String )
    self.pub_land = rospy.Publisher( "ardrone/land", Empty )
    self.pub_takeoff = rospy.Publisher( "ardrone/takeoff", Empty )
    self.pub_emergency = rospy.Publisher( "ardrone/reset", Empty )
    self.pub_yaw = rospy.Publisher( "/cmd_vel", Twist )
    
    #subscribers
    self.sub_joy = rospy.Subscriber( "joy", Joy, self.handle_joy )
    self.sub_nav = rospy.Subscriber( "ardrone/navdata", Navdata, self.handle_navdata )
    self.sub_log = rospy.Subscriber( "yudrone/log", String, self.handle_log )
    self.sub_image = rospy.Subscriber(imageTopic, Image,self.handle_image)
    rospy.sleep(0.1)
    
    #variables
    self.camState='front'
    
    #stage 7: finalize ............................................
    self.OnRBInput()
    self.OnRBVideo()
    self.pub_log.publish('ROS initialized')
    
  def openDriver(self):
    '''
    opens ardrone_driver in a subprocess
    '''
    rospy.loginfo("open driver...")
    cmd= ROSDIR + "rosrun ardrone_autonomy ardrone_driver"
    self.pipe_ros=subprocess.Popen(['gnome-terminal', '--command='+cmd])
    rospy.sleep(10)
  
  def openRoscore(self):
    '''
    opens ardrone_driver in a subprocess
    '''
    rospy.loginfo('open roscore...')
    cmd=ROSDIR + "roscore"
    self.pipe_ros=subprocess.Popen(['gnome-terminal', '--command='+cmd])
    rospy.sleep(3)
    
  def openJoyNode(self):
    '''
    opens joy_node in a subprocess
    '''
    rospy.loginfo('open joynode...')
    rospy.set_param("joy_node/dev", "/dev/input/js2")
    cmd=ROSDIR + "rosrun joy joy_node"
    self.pipe_joy=subprocess.Popen(['gnome-terminal', '--command='+cmd])
    rospy.sleep(3)
    
  def openArToolkit(self):
    '''
    opens ar_recog in a subprocess
    '''
    rospy.loginfo('open ar_toolkit...')
    rospy.set_param('aov', 0.67)
    cmd=ROSDIR + "rosrun ar_recog ar_recog image:=/ardrone/image_raw"
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
    #rospy.loginfo('updateimage')
    img = wx.ImageFromData(data.width, data.height, data.data)
    bitmap = wx.BitmapFromImage(img)
    self.imageView.SetBitmap(bitmap)
    
  def OnRBVideo(self, event='NONE'):
    '''
    Callback function for changing video radiobuttons
    '''
    if self.rbFront.GetValue() == True and self.camState == 'bottom':
      subprocess.call(["rosservice","call","/ardrone/togglecam"])
      self.camState = 'front'
      self.pub_log.publish('Frontcam activated')
    if self.rbBottom.GetValue() == True and self.camState == 'front':
      subprocess.call(["rosservice","call","/ardrone/togglecam"])
      self.camState = 'bottom'
      self.pub_log.publish('Bottomcam activated') 
    
  '''************************************************************************************************************************
  DIAGNISTICS
  ************************************************************************************************************************'''  
  def setCommands(self, shell):
   self.shell = shell 
    
  def handle_navdata(self, navdata):
    '''
    Callback function for navigation data comming from ardrone
    '''
    self.txtConnection.SetLabel('Connection: GOOD') #TODO put in watchdog
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
      currStatus='emergency'
    self.txtState.SetLabel('Status: ' + currStatus)
    self.txtAltd.SetLabel('Altitude (mm): ' + str(navdata.altd))
    self.droneState = navdata.state
    #print(navdata)
    if self.shell != None:
      wx.CallAfter(shell.update, navdata) # call asynchon
	
  def handle_log(self, log):
    '''
    Callback function for "yudrone/log"
    TODO: describe what it does
    '''
    rospy.loginfo(log.data)
    
  '''************************************************************************************************************************
  INPUT
  ************************************************************************************************************************'''
  def OnRBInput(self, event='NONE'):
    '''
    Callback function for changing input radiobuttons
    '''
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
    '''
    Callback function for incomming joypad commands
    '''
    
      # btn nr  for takeoff
    if joy.buttons[3]==1: # and self.rbJoypad.GetValue() == True:# and status["state"] == 'ground':
      self.pub_log.publish('start command')
      self.pub_takeoff.publish( Empty() )
      rospy.sleep(0.1)
    # btn nr  for land
    if joy.buttons[1]==1:# and self.rbJoypad.GetValue() == True:# and status["state"] == 'flight':
      self.pub_log.publish('land command')
      self.pub_land.publish( Empty() )
      rospy.sleep(0.1)
    # btn nr  for emergency
    if joy.buttons[0]==1:
      if self.droneState != 0:
	self.pub_log.publish('emergency mode on')
      else:
	self.pub_log.publish('emergency mode off')
      self.pub_emergency.publish( Empty() )
      rospy.sleep(0.1)
    # btn for stop
    if joy.buttons[2]==1:
      yaw = Twist()
      self.pub_yaw.publish(yaw)
      
    # joysticks  
    if self.rbJoypad.GetValue() == True:
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
    
    '''
    Callback function for selecting command from combobox
    '''
    item = event.GetSelection()
    self.cmdShell.setFocus()
    self.cmdShell.clearCommand()
    self.cmdShell.write("shell." + self.CommandList[item])
    self.cmdShell.autoCallTipShow("shell." + self.CommandList[item])
    
  
'''************************************************************************************************************************
top-level-code
STARTS application
************************************************************************************************************************'''
if __name__ == '__main__':
  try:
    # start application
    app = wx.App(redirect=False)
    flight = Flight()
    shell = Commands(flight)
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
    print 'Exception occured'
    for line in sys.exc_info():
      print(line)

    