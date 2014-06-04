#!/usr/bin/python
from cuiTool import *
def Run(t,args):
  print '###CAUTION:',t.ArmStr(),'arm is relaxed'
  t.ActivateMannController()
  while True:
    print 'Move',t.ArmStr(),'arm to preferred point'
    print 'Is it OK?'
    if AskYesNo():
      break
  print '###CAUTION:',t.ArmStr(),'arm is fixed'
  t.ActivateStdController()
