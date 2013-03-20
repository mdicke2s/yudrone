Header header

#*****************************************************************************
# This file contains message description to send different commands.
# Different purposes are enabled by setting hasXYZ to True.
#*****************************************************************************
# Project:	yudrone
# Author:	Michael Dicke
# Repository:	https://github.com/mdicke2s/yudrone
#*****************************************************************************

# publishes Twist msg ********************************************************
bool hasTwist
geometry_msgs/Twist twist

# Altitude(self, delta) ******************************************************
bool hasAltd
int32 altd

# MaxAltd(self, val) *********************************************************
bool hasMaxAltd
int32 MaxAltd

# MinAltd(self, val) *********************************************************
bool hasMinAltd
int32 MixAltd
      
# Yaw(self, angle) ***********************************************************
bool hasYaw
int32 yaw

# Horizontal(self, x, y) *****************************************************
bool hasHorizontal
int32 horizontalX
int32 horizontalY

# Face(self, tagNr, faceYaw = True, faceDist = True, faceAlt = True,  ********
# facePerpend = True, keepSearching = False) 
bool hasFace
int32 tagNr

# Release(self) **************************************************************
bool hasRelease

# Search(self, tagNr) ********************************************************
bool hasSearch
# int32 tagNr already defined

# Approach(self, tagNr) ******************************************************
bool hasApproach
# int32 tagNr already defined

# Navigate(self, x, y, z, xRot, yRot, zRot) **********************************
bool hasNavigate
int32[] nav_XYZ_RxRyRz

# YawSpeed(self, val) ********************************************************
bool hasYawSpeed
int32 yawSpeed

# HrzSpeed(self, val) ********************************************************
bool hasHrzSpeed
int32 hrzSpeed

# not implemented by now...
# TakeOff(self): Land(self): ToggleEmerg(self) *******************************
# __reset_twist(self, delay = 0) *********************************************

