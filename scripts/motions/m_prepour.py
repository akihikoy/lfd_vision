#!/usr/bin/python
from core_tool import *
def Help():
  return '''Pre-pouring motion where the bottle should be grabbed.
  Usage: prepour BOTTLE_ID, CUP_ID
    BOTTLE_ID: identifier of bottle. e.g. 'b1'
    CUP_ID: identifier of cup. e.g. 'c1' '''
def Run(t,args=()):
  bottle= args[0]
  cup= args[1]

  if not 'grabbed' in t.attributes[bottle]:
    print 'Error: not grabbed: ',bottle
    return

  #Pouring edge point in the wrist frame (control point)
  lb_x_pour_e= t.attributes[bottle]['l_x_pour_e']
  t.ExecuteMotion('infer',(bottle,'x'))
  x_b= t.attributes[bottle]['x']
  x_w= t.CartPos()
  lw_x_pour_e= TransformLeftInv(x_w, Transform(x_b,lb_x_pour_e))
  print 'lw_x_pour_e=',VecToStr(lw_x_pour_e)

  #Pouring location on the cup frame:
  t.ExecuteMotion('infer',(cup,'x'))
  x_c= t.attributes[cup]['x']
  lc_x_pour_l= t.attributes[cup]['l_x_pour_l']
  x_pour_l= Transform(x_c,lc_x_pour_l)
  x_pour_l[3:7]= t.attributes[bottle]['q_pour_start']


  #Move upward
  x_w= t.CartPos()
  x_w[2]+= 0.05
  t.MoveToCartPos(x_w,2.0,[],True)

  #Move pouring edge to pouring location
  print 'x_pour_l=',x_pour_l
  t.MoveToCartPos(x_pour_l,3.0,lw_x_pour_e,True)

