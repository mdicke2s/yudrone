#!/usr/bin/env python
import roslib;
import rospy
import subprocess
import time
import os
import wx
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class Flight:
      
  def main():
    land_pub = rospy.Publisher( "ardrone/land", Empty )
    takeoff_pub = rospy.Publisher( "ardrone/takeoff", Empty )
    reset_pub = rospy.Publisher( "ardrone/reset", Empty )
    cmd_vel_pub = rospy.Publisher( "cmd_vel_interm", Twist )
    
    #open driver
    #print("+++ OPENING DRIVER ++++++++++++++++++++++++++++")
    #subprocess.Popen(["rosrun","ardrone_autonomy","ardrone_driver"])
    #time.sleep(7)
    #raw_input("Any key to continue...")
    
    #open camera windows, diagnostics
    print("+++ OPENING DIAGNISTICS ++++++++++++++++++++++++")
    subprocess.Popen(["rosrun","image_view", "image_view", "image:=/ardrone/front/image_raw"])
    #subprocess.Popen(["rosrun","image_view", "image_view", "image:=/ardrone/bottom/image_raw"])
    #subprocess.Popen(["rostopic","echo","/ardrone/navdata"])
    
    rospy.init_node('yudrone_flight')
    cmd = Twist()
    while 1:
    #  subprocess.call(["rosservice","call","/ardrone/togglecam"])
    #  subprocess.call(["rosservice","call","/ardrone/togglecam"])
    #  time.sleep(5)
      abcd = input("linZ Val: ")
      cmd.angular.x = cmd.angular.y = 0
      cmd.angular.z = 0
      cmd.linear.z = abcd * 50
      cmd.linear.x = 0
      cmd.linear.y = 0
      cmd_vel_pub.publish( cmd )
    
  main()