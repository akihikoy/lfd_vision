#!/usr/bin/python
from cuiTool import *
def Run(t,args):
  print t.material_amount
  l_cf_e= t.control_frame[t.whicharm] #Local vector to the current control frame
  t.FlowAmountControl(0.01, [1,0,0], x_ext=l_cf_e, max_duration=20.0)
