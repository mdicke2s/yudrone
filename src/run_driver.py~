#!/usr/bin/env python
import roslib;
import rospy
import subprocess, time, os, wx, sys, getopt
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class Flight():
 
  def __init__(self):
    #initialization
    self.land_pub = rospy.Publisher( "ardrone/land", Empty )
    self.takeoff_pub = rospy.Publisher( "ardrone/takeoff", Empty )
    self.reset_pub = rospy.Publisher( "ardrone/reset", Empty )
    self.cmd_vel_pub = rospy.Publisher( "cmd_vel_interm", Twist )
    
    # parse args
    self.config = {"driver":0,"battery":0,"nav":0,"front":0,"bottom":0}
    
    try:
      opts, args = getopt.getopt(sys.argv[1:],"dbnfbau")
    except getopt.GetoptError:
      self.printUsage()
      sys.exit(2)
    for opt, arg in opts:
      if opt == '-h':
         self.printUsage()
         sys.exit()
      elif opt == '-d':
         self.config["driver"] = 1
      elif opt == '-b':
         self.config["battery"] = 1
      elif opt == '-n':
         self.config["nav"] = 1
      elif opt == '-f':
         self.config["front"] = 1
      elif opt == '-b':
         self.config["bottom"] = 1
      elif opt == '-a':
         self.config["driver"] = 1
         self.config["battery"] = 1
         self.config["nav"] = 1
         self.config["front"] = 1
         self.config["bottom"] = 1
      elif opt == '-u':
         self.config["nav"] = 1         
         self.config["battery"] = 1
    self.stages()
  
  def printUsage(self):
    print 'flight.py [-d] [-b] [-n] [-f] [-b] [-a] [-u]'
    print 'Navigates Ardrone 2.0 quadRotor using ROS and ardrone_autonomy'
    print '-d\topens driver'
    print '-b\topens battery monitor'
    print '-n\topens navdata monitor'
    print '-f\topens bottom camera view'
    print '-a\topens all of them'
    print '-u\t"usual"'
         
  def openDriver(self):
    print("+++ OPENING DRIVER ++++++++++++++++++++++++++++")
    subprocess.Popen(["rosrun","ardrone_autonomy","ardrone_driver"])
    time.sleep(10)
    raw_input("Any key to continue...")
    
  def stages(self):
    rospy.init_node('yudrone_flight')
    
    #open driver
    if (self.config["driver"] == 1):
      self.openDriver()
    
    #open camera windows, diagnostics
    print("+++ OPENING DIAGNISTICS ++++++++++++++++++++++++")
    #if (self.config["battery"] == 1):
      
    if (self.config["nav"] == 1):
      subprocess.Popen(["rostopic","echo","/ardrone/navdata"])
    if (self.config["front"] == 1):
      subprocess.Popen(["rosrun","image_view", "image_view", "image:=/ardrone/front/image_raw"])
    if (self.config["bottom"] == 1):
      subprocess.Popen(["rosrun","image_view", "image_view", "image:=/ardrone/bottom/image_raw"])
    
    print("+++ OPENING NAVIGATION +++++++++++++++++++++++++")
    cmd = Twist()
    while 0:
    #  subprocess.call(["rosservice","call","/ardrone/togglecam"])
    #  subprocess.call(["rosservice","call","/ardrone/togglecam"])
    #  time.sleep(5)
      abcd = input("linZ Val: +")
      cmd.angular.x = cmd.angular.y = 0
      cmd.angular.z = 0
      cmd.linear.z += 50
      cmd.linear.x = 0
      cmd.linear.y = 0
      cmd_vel_pub.publish( cmd )
      
Flight()