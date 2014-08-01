#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move left arm/gripper to calibration posture.
  Usage: initc'''
def Run(t,args=()):
  t.SwitchArm(1) #Left arm

  #angles= [0.8, 0.0, 1.57, -1.5, 3.14, 0,0]
  angles= [0.2943815760479809, 0.4523002598851919, 1.7590782665266982, -1.5979523190896374, 16.15402651639962, -0.9139384808240009, 0.0]
  #t.CommandGripper(0.08,50,blocking=False)
  t.MoveToJointPos(angles,dt=3.0,blocking=False)

  t.CommandGripper(0.005,50,blocking=False)
  print 'Insert the calibration stick. Ready?'
  if AskYesNo():
    t.CommandGripper(0.0,50,blocking=False)

