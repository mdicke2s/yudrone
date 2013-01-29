'''----------------------------------------------------------------------------
This file contains all commands used from the command line interface.
A python console is embedded in the yudrone gui, that performs these commands.
-------------------------------------------------------------------------------
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
----------------------------------------------------------------------------'''
pub_takeoff = 0
def initCommands(takeoff):
  pub_takeoff = takeoff

def Altitude(delta):
  '''
  Action:
    In-/decreases the UAVs hovering altitude by a  certain value. The function uses the scale given positive means up by the IMUs altimeter.
  Parameter:
    delta [mm] (integer)
  '''
  print('Altitude set to ' + str(delta))
  
  
def MaxAlt(val):
  '''
  Action:
    Sets threshold value. Once defined, thefunction will not exceed those values any more.
    If not defined, the maximal altitude is limited by a firmware defined threshold.
  Parameter:
    val [mm]
  '''
  print('MaxAlt set to ' + str(val))
  
def MinAlt(val):
  '''
  Action:
    Sets threshold values. Once defined, thefunction will not exceed those values any more.
    If not defined, the maximal altitude is limited by the ground.
  Parameter:
    val [mm]
  '''
  print('MinAlt set to ' + str(val))
  
def Yaw(angle):
  '''
  Action:
    Rotates the UAV by a specified angle (yaw).
    The device will turn slowly until the designated angle is reached.
    Therefore it accesses the rotation angle provided by the internal compass.
  Parameter:
    angle [deg] (integer) -180, ..., +180 negative is clockwise 
  '''
  print('Yaw ' + str(angle))  
  
def YawSpeed(val):
  '''
  Action:
    Sets the speed to be used performing Yaw().
  Parameter:
    val = 1, 2, ..., 255
  '''
  print('YawSpeed set to ' + str(val))
  
def Horizontal(x, y):
  '''
  Action:
    Moves the UAV a certain distance in a specified horizontal direction (see figure 1).
    The distance cannot be determined exactly, but it is approximated by experienced data
    and the horizontal speed.
  Parameter:
    x[cm], y[cm] (both integer)
  '''
  print('Altitude set to ' + str(val))
  
def HrzSpeed(val):
  '''
  Action:
    Sets the speed to be used performing Horizontal().
  Parameter:
    1, 2, ..., 255
  '''
  print('HrzSpeed set to ' + str(val))
  
def TakeOff():
  '''
  Action:
    Triggers built-in takeoff command.    
  '''
  print('TakeOff cmd')
  pub_takeoff.publish( Empty() )
  
def Land():
  '''
  Action:
    Triggers built-in land command.    
  '''
  print('Land cmd')
  
def Pause():
  '''
  Action:
    Interrupts the current command. Any incoming command will be ignored during pause.
  '''
  print('Pause cmd')
  
def Continue():
  '''
  Action:
    Continues execution of current command and exits pause mode.
  '''
  print('Continue cmd')
  
def Break():
  '''
  Action:
    Same as pause, but additionally the current action will be dismissed.   
  '''
  print('Break cmd')
  
def Batch(fileName):
  '''
  Action:
    Runs the specified batch-file.
  Parameter:
    fileName (string)
  '''
  print('Run ' + str(fileName))  
  
def Exit():
  '''
  Action:
    Cancels a batch run.
  '''
  print('Cancel cmd')
  
def ToggleEmerg():
  '''
  Action:
    Toggles the emergency mode of Ardrone. Red LEDs indicate emergency mode, where all
    rotors are stopped. Without emergency these LEDs show up green.
  '''
  print('Emegr cmd')
  
def Face(tagNr):
  '''
  Action:
    Faces a specified tag. As long as this tag is visible in the field of view, the Ardrone will
    automatically yaw towards the target. This function overrides yaw control, whereas
    horizontal motion is not affected. Whenever the system loses the target, it will prompt a warning.
  Parameter:
    tagNr (integer)
  '''
  print('Face cmd')
  
def Release():
  '''
  Action:
    Releases facing tag specified by Face().
  '''
  print('release cmd')
  
def Search(tagNr):
  '''
  Action:
    Ardrone will yaw slowly, until the specified tag [direction (string)] appears. Overrides all movement controls.
    If it rotated full 360 without finding the target, a warning is prompted.
    [The optional parameter direction may be set "-",changing the direction to "clockwise"]
  Parameter:
    tagNr (integer)
  '''
  print('Searching ' + str(tagNr))
  
def Approach(tagNr):
  '''
  Action:
    Ardrone will automatically search and approach the specified tag as described in 3.2.
    Overrides all movement controls.
  Parameter:
    tagNr (integer)
  '''
  print('Approaching ' + str(tagNr)) 
  
def Navigate(x, y, z, xRot, yRot, zRot):
  '''
  Action:
    Performs a graph-based navigation as described in 3.3. Overrides all movement controls.
    Thisfunction it will require an initialization of the graph environment.
  Parameter:
    x, y, z,xRot, yRot, zRot (all integer) 
  '''
  print('Navigate cmd') 
 
