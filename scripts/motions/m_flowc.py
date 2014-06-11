#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test the flow amount controller.
  Usage: flowc'''
def Run(t,args=[]):
  print t.material_amount
  l_cf_e= t.control_frame[t.whicharm] #Local vector to the current control frame
  xe_init= t.CartPos(l_cf_e)  #Store the current position to move back later
  #print 'l_cf_e=',VecToStr(l_cf_e)

  #Flow amount control
  amount_trg= 0.03
  axis= [1,0,0]
  max_theta= math.pi*0.8
  t.FlowAmountControl(amount_trg, axis, max_theta, x_ext=l_cf_e, trg_duration=8.0, max_duration=10.0)

  print 'Moving back to the initial pose...'
  t.MoveToCartPosI(xe_init,3.0,x_ext=l_cf_e,inum=30,blocking=True)

