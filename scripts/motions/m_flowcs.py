#!/usr/bin/python
from core_tool import *
def Help():
  return '''Test the flow amount controller with shaking.
  Usage: flowcs'''
def Run(t,args=[]):
  print t.material_amount
  l_cf_e= t.control_frame[t.whicharm] #Local vector to the current control frame
  #print 'l_cf_e=',VecToStr(l_cf_e)
  amount_trg= 0.035
  t.FlowAmountControlWithShaking(amount_trg, x_ext=l_cf_e, max_duration=25.0)
