#!/usr/bin/python
from cuiTool import *
def Run(t,args):
  print t.material_amount
  l_cf_e= t.control_frame[t.whicharm] #Local vector to the current control frame
  #print 'l_cf_e=',VecToStr(l_cf_e)
  amount_trg= 0.01
  t.FlowAmountControlWithShaking(amount_trg, x_ext=l_cf_e, max_duration=10.0)
