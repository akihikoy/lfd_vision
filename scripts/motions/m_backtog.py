#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm back to the position where it grabbed the object.
  Usage: backtog OBJ_ID [, HAND]
    OBJ_ID: identifier of object. e.g. 'b1'
    HAND: 'l': left hand, 'r': right hand (default: 'l')'''
def Run(t,args=()):
  obj= args[0]

  if not 'grabbed' in t.attributes[obj]:
    print 'Error: not grabbed: ',obj
    return

  whicharm= t.whicharm
  handid= t.attributes[obj]['grabbed']['grabber_handid']
  t.SwitchArm(handid)

  #t.CommandGripper(0.08,50.0,True)
  #del t.attributes[obj]['grabbed']

  if 'joint_angles' in t.attributes[obj]['grabbed']:
    target= t.attributes[obj]['grabbed']['joint_angles']
    print 'Move back to q=',target
    t.mu.arm[handid].moveToJointAngle(target, 4.0)
  else:
    print 'Error: joint_angles is not assigned: ',obj

  t.SwitchArm(whicharm)

