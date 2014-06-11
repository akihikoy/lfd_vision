#!/usr/bin/python
from core_tool import *
import time
def Help():
  return '''Register a base point (bp) by moving gripper.
  Usage: setbp BP_ID'''
def Run(t,args=[]):
  bpid= args[0]
  t.CommandGripper(0.0,50,True)
  print '###CAUTION:',t.ArmStr(),'arm is relaxed'
  t.ActivateMannController()
  print 'Move',t.ArmStr(),'arm to a point that you want to register'
  print 'Is it OK?'
  if AskYesNo():
    print '###CAUTION:',t.ArmStr(),'arm is fixed'
    t.ActivateStdController()
    time.sleep(0.5)  # Wait that the CartPos observation is updated
    xe= t.CartPos(t.control_frame[t.whicharm])
    print 'xe= ',VecToStr(xe)
    print 'Do you want to change the orientation to [0,0,0,1]?'
    if AskYesNo():
      xe[3:7]= [0.0,0.0,0.0,1.0]
    t.base_x[bpid]= xe
    print 'bp add',bpid,VecToStr(t.base_x[bpid])
    print 'Done'
  else:
    print '###CAUTION:',t.ArmStr(),'arm is fixed'
    t.ActivateStdController()
    print 'Canceled'
