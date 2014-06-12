#!/usr/bin/python
from core_tool import *
import copy
def Help():
  return '''Script to pour.
  Usage: pour BOTTLE_ID CUP_ID
    BOTTLE_ID: identifier of bottle. e.g. b1
    CUP_ID: identifier of cup. e.g. c1'''
def Run(t,args=[]):
  bottle= args[0]
  cup= args[1]

  #Use left hand
  whicharm= t.whicharm
  t.SwitchArm(1)

  #Current control point in the wrist frame
  #lw_xe= t.control_frame[t.whicharm]
  #x_b= t.attributes[bottle]['x'] #Bottle base pose on the torso frame
  #if len(x_b)!=7:
    #print 'Bottle ',bottle,' pose is not observed'
    #t.SwitchArm(whicharm)
    #return

  x_c= t.attributes[cup]['x'] #Cup base pose on the torso frame
  if len(x_c)!=7:
    print 'Cup',cup,' pose is not observed'
    t.SwitchArm(whicharm)
    return

  #Grab pose on the bottle frame:
  #lb_x_grab= t.attributes[bottle]['l_x_grab']
  #Grab pose on the torso frame
  #x_grab= Transform(x_b,lb_x_grab)
  #print 'x_grab=',VecToStr(x_grab)
  #t.CommandGripper(t.attributes[bottle]['g_pre'],50,True)
  #x_grab0= copy.deepcopy(x_grab)
  #x_grab0[0]= t.CartPos(lw_xe)[0]
  #t.MoveToCartPos(x_grab0,3.0,lw_xe,True)
  #t.MoveToCartPos(x_grab,3.0,lw_xe,True)

  t.ExecuteMotion('pregrab',[bottle,'l'])

  #t.CommandGripper(0.0,t.attributes[bottle]['f_grab'],True)

  t.ExecuteMotion('grab',[bottle,'l'])


  #Infere bottle pose
  t.ExecuteMotion('infer',[bottle,'x'])
  x_b= t.attributes[bottle]['x']
  if len(x_b)!=7:
    print 'Bottle ',bottle,' pose is not observed'
    t.SwitchArm(whicharm)
    return

  #Pouring edge point on the bottle frame:
  lb_x_pour_e= t.attributes[bottle]['l_x_pour_e']

  x_w= t.CartPos()
  #Pouring edge point in the wrist frame
  lw_x_pour_e= TransformLeftInv(x_w, Transform(x_b,lb_x_pour_e))
  print 'lw_x_pour_e=',VecToStr(lw_x_pour_e)


  #Move upward
  x_w= t.CartPos()
  x_w[2]+= 0.10
  t.MoveToCartPos(x_w,2.0,[],True)

  #Pouring location on the cup frame:
  lc_x_pour_l= t.attributes[cup]['l_x_pour_l']
  x_pour_l= Transform(x_c,lc_x_pour_l)

  #Pouring orientation:
  x_pour_l[3:7]= t.attributes[bottle]['q_pour_start']

  #Move pouring edge to pouring location
  print 'x_pour_l=',VecToStr(x_pour_l)
  t.MoveToCartPos(x_pour_l,3.0,lw_x_pour_e,True)

  #Pouring orientation:
  #pourq= [0.789370121064, 0.0178832059046, -0.0679767323576, 0.609880452856]

  #pourexecx= x_pour_l
  #pourexecx[3:7]= pourq  #Only change the orientation

  #print 'pourexecx=',VecToStr(pourexecx)
  #t.MoveToCartPosI(pourexecx,4.0,lw_x_pour_e,30,True)

  t.SwitchArm(whicharm)

