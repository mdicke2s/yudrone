#!/usr/bin/env python

'''************************************************************************************************************************
This file initializes default parameter for yudrone usage
***************************************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
************************************************************************************************************************'''

# ros
import roslib;
roslib.load_manifest('yudrone')
import rospy

def setDefaultParameter():
  # set default parameter for yudrone
  rospy.set_param("joy_node/dev", '/dev/input/js2')
  rospy.set_param('yudrone/ARDRONE_IP', '192.168.1.1')
  rospy.set_param('yudrone/ROSDIR', "/opt/ros/fuerte/bin/")
  rospy.set_param('yudrone/ARRECOGDIR', '/home/viki/ros_workspace/ar_recog/bin')
  rospy.set_param('aov', 0.67)
  rospy.set_param('yudrone/maxAltitude', 2000)
  rospy.set_param('yudrone/minAltitude', 100)
  rospy.set_param('yudrone/yawSpeed', 100)
  rospy.set_param('yudrone/hrzSpeed', 100)
  rospy.loginfo('parameter were successfully initialized')

'''************************************************************************************************************************
top-level-code
************************************************************************************************************************'''
if __name__ == '__main__':
  try:
    rospy.init_node('def_param')
    setDefaultParameter()
  except:
    print 'Exception occured'
    for line in sys.exc_info():
      print(line)