#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move left arm/gripper to init posture.
  Usage: init0'''
def Run(t,args=[]):
  whicharm= t.whicharm
  t.SwitchArm(0)

  #angles= [0.560346004529, 0.406872786178, 1.69814343251, -1.89632475525, -28.3134341521, -1.48083122174, -4.64978978552]
  angles= [0.603789072024, 0.471672497276, 1.85064087238, -2.09393677629, -28.1654034566, -1.72261029073, -4.76443578999]
  t.CommandGripper(0.08,50,blocking=False)
  t.MoveToJointPos(angles,dt=3.0,blocking=False)

  t.SwitchArm(whicharm)
