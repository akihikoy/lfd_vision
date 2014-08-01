#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move left arm/gripper to calibration posture.
  Usage: initc'''
def Run(t,args=()):
  t.SwitchArm(1) #Left arm

  angles= [0.8, 0.0, 1.57, -1.5, 3.14, 0,0]
  #t.CommandGripper(0.08,50,blocking=False)
  t.MoveToJointPos(angles,dt=3.0,blocking=False)

