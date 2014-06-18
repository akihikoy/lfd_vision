#!/usr/bin/python
from core_tool import *
def Help():
  return '''Release an object from gripper.
  Usage: release OBJ_ID
    OBJ_ID: identifier of object. e.g. 'b1' '''
def Run(t,args=()):
  obj= args[0]

  if not 'grabbed' in t.attributes[obj]:
    print 'Error: not grabbed: ',obj
    return

  whicharm= t.whicharm
  t.SwitchArm(t.attributes[obj]['grabbed']['grabber_handid'])

  t.CommandGripper(0.08,50.0,True)
  del t.attributes[obj]['grabbed']

  t.SwitchArm(whicharm)

