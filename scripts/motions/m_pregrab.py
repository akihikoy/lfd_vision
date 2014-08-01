#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move to an object to grab.
  Usage: pregrab OBJ_ID [, HAND]
    OBJ_ID: identifier of object. e.g. 'b1'
    HAND: 'l': left hand, 'r': right hand (default: 'l')'''
def Run(t,args=()):
  obj= args[0]
  hand= 'l'
  if len(args)>=2:  hand= args[1]
  handid= 0 if hand=='r' else 1
  whicharm= t.whicharm
  t.SwitchArm(handid)

  #Gripper pose in the wrist frame
  lw_xe= t.attributes['wrist_'+hand]['l_x_gripper']
  x_o= t.attributes[obj]['x'] #Object's pose on the torso frame

  if len(x_o)!=7:
    print 'Object',obj,' pose is not observed'
    t.SwitchArm(whicharm)
    return

  #Grab pose on the object's frame:
  lo_x_grab= t.attributes[obj]['l_x_grab']
  #Grab pose on the torso frame
  x_grab= Transform(x_o,lo_x_grab)
  print 'x_grab=',VecToStr(x_grab)
  x_grab0= copy.deepcopy(x_grab)
  x_grab0[0]-= t.attributes[obj]['g_width']

  #Open the gripper to 'g_pre' value
  t.CommandGripper(t.attributes[obj]['g_pre'],50,True)
  #Move the gripper to front of the object
  t.MoveToCartPos(x_grab0,3.0,lw_xe,True)
  #Move the gripper to the grab pose
  t.MoveToCartPos(x_grab,3.0,lw_xe,True)

  t.SwitchArm(whicharm)
