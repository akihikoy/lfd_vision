#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move to an object to grab.
  Usage: pregrab OBJ_ID [, HAND]
    OBJ_ID: identifier of object. e.g. 'b1'
    HAND: 'l': left hand, 'r': right hand (default: 'l')'''
class TLocal:
  what_is_this= 'This is an object to store variables shared in local functions.'
def Run(t,args=()):
  obj= args[0]
  hand= 'l'
  if len(args)>=2:  hand= args[1]
  handid= 0 if hand=='r' else 1
  whicharm= t.whicharm
  t.SwitchArm(handid)

  l= TLocal()

  def infer_x_grab():
    #Gripper pose in the wrist frame
    l.lw_xe= t.attributes['wrist_'+hand]['l_x_gripper']
    t.ExecuteMotion('infer',(obj,'x'))
    x_o= t.attributes[obj]['x'] #Object's pose on the torso frame
    if len(x_o)!=7:
      print 'Object',obj,' pose is not observed'
      t.SwitchArm(whicharm)
      return

    #Grab pose on the object's frame:
    lo_x_grab= t.attributes[obj]['l_x_grab']
    #Grab pose on the torso frame
    l.x_grab= Transform(x_o,lo_x_grab)
    print 'x_grab=',l.x_grab
    l.x_grab0= copy.deepcopy(l.x_grab)
    l.x_grab0[0]-= t.attributes[obj]['g_width']  #TODO: generalize
    l.x_grab0[3:7]= t.CartPos()[3:7]

  infer_x_grab()
  x_grab0= copy.deepcopy(l.x_grab0)  #Store value to use later

  #Open the gripper to 'g_pre' value
  t.CommandGripper(t.attributes[obj]['g_pre'],50,True)
  #Move the gripper to front of the object
  t.MoveToCartPos(l.x_grab0,3.0,l.lw_xe,True)

  #>>>FIXME
  #Infer again to adjustment by the marker on the wrist
  ar_adjust_ratio= t.ar_adjust_ratio
  t.ar_adjust_ratio= 0.2
  #for i in range(20):
    #print t.ar_adjust_err
    #time.sleep(0.5)
  #print 'OK?'
  #while not AskYesNo():
    #print t.ar_adjust_err
  time.sleep(0.5)
  wait_counter=0
  while t.ar_adjust_err>0.005 and wait_counter<50:
    print 'Waiting AR marker adjustment..',t.ar_adjust_err
    time.sleep(0.5)
    wait_counter+=1
  t.ar_adjust_ratio= ar_adjust_ratio

  infer_x_grab()
  #Move again if the adjustment is large
  adjustment= max([abs(l.x_grab0[d]-x_grab0[d]) for d in range(7)])
  print 'Err:',adjustment,[abs(l.x_grab0[d]-x_grab0[d]) for d in range(7)]
  if adjustment>0.01:
    t.MoveToCartPos(l.x_grab0,1.0,l.lw_xe,True)

  #<<<FIXME


  #Move the gripper to the grab pose
  t.MoveToCartPos(l.x_grab,3.0,l.lw_xe,True)

  t.SwitchArm(whicharm)
