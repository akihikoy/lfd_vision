#!/usr/bin/python
from core_tool import *
def Help():
  return '''Infer attributes.
  Usage: infer OBJ_ID, ELEM_ID'''
def Run(t,args=()):
  obj= args[0]
  elem= args[1]

  #Infer x when obj is grabbed
  if elem=='x' and 'grabbed' in t.attributes[obj]:
    grabber_handid= t.attributes[obj]['grabbed']['grabber_handid']
    if grabber_handid!=t.whicharm:
      whicharm= t.whicharm
      t.SwitchArm(grabber_handid)
    else:
      whicharm= -1

    x_w= t.CartPos()
    grabber_wrist= t.attributes[obj]['grabbed']['grabber_wrist']
    lw_xe= t.attributes[grabber_wrist]['l_x_gripper']
    lo_x_grab= t.attributes[obj]['l_x_grab']
    t.attributes[obj][elem]= TransformRightInv( Transform(x_w, lw_xe), lo_x_grab )
    print 'Inferred: [',obj,'][',elem,']= ',t.attributes[obj][elem]

    if whicharm!=-1:
      t.SwitchArm(whicharm)
    return

  print 'Not inferred: [',obj,'][',elem,']'
