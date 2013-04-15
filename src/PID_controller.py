'''************************************************************************************************************************
simple PID control
***************************************************************************************************************************
Project:	yudrone
Author:		Michael Dicke
Repository:	https://github.com/mdicke2s/yudrone
************************************************************************************************************************'''


class PID_controller:
  def __init__(self, currReference = 0.0, pGain = 0.5, iGain = 0.01, dGain = 0.01, maxIntegral = 0.5):
    self.setReferece(currReference)
    self.setPGain(pGain)
    self.setIGain(iGain)
    self.setDGain(dGain)
    self.maxIntegral = maxIntegral
    
    self.reset()
    
  def setPGain(self, pGain):
    self.pGain = pGain
    
  def setIGain(self, iGain):  
    self.iGain = iGain
    
  def setDGain(self, dGain):
    self.dGain = dGain
    
  def setReferece(self, currReference):
    self.reference = currReference
    
  def reset(self):
    self.prev_error = 0.0
    self.integral = 0.0
    
  def update(self, currentValue):
    error = self.reference - currentValue
    if abs(self.integral) < self.maxIntegral:
      self.integral = self.integral + error
    
    P_value = self.pGain * error
    I_value = self.iGain * self.integral
    D_value = self.dGain * (error - self.prev_error)
    
    self.prev_error = error
    print('IN: ref(%3f) val(%3f) OUT: p(%3f) i(%3f) d(%3f)'%(self.reference, currentValue, P_value, I_value, D_value))
    return P_value + I_value + D_value