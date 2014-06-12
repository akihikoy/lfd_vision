#!/usr/bin/python
from core_tool import *
def Help():
  return '''Grab an object.
  Usage: grab OBJ_ID [HAND]
    OBJ_ID: identifier of object. e.g. b1
    HAND: l: left hand, r: right hand (default: l)'''
def Run(t,args=[]):
  obj= args[0]
  hand= 'l'
  if len(args)>=2:  hand= args[1]
  handid= 0 if hand=='r' else 1
  whicharm= t.whicharm
  t.SwitchArm(handid)

  if 'grabbed' in t.attributes[obj]:
    print 'Error: already grabbed: ',obj
    t.SwitchArm(whicharm)
    return

  t.CommandGripper(0.0,t.attributes[obj]['f_grab'],True)
  t.attributes[obj]['grabbed']= {}
  t.attributes[obj]['grabbed']['grabber']= 'gripper_'+hand
  t.attributes[obj]['grabbed']['grabber_wrist']= 'wrist_'+hand
  t.attributes[obj]['grabbed']['grabber_hand']= hand
  t.attributes[obj]['grabbed']['grabber_handid']= handid

  t.SwitchArm(whicharm)

