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
bool hasAltdDelta
int32 altdDelta

# Altitude(self, abs) *******************************************************
bool hasAltdAbs
int32 altdAbs

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

# Search(self, tagNr, startAltd=0) *******************************************
bool hasSearch
# int32 tagNr already defined
int32 searchStartAltd

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

# TakeOff(self, tagNr=None) **************************************************
bool hasTakeoff
# int32 tagNr already defined

# Land(self, tagNr=None) *****************************************************
bool hasLand
# int32 tagNr already defined

# __unlock(self) *************************************************************
bool forceUnlock

# not implemented by now...
# ToggleEmerg(self) *******************************
# __reset_twist(self, delay = 0) *********************************************